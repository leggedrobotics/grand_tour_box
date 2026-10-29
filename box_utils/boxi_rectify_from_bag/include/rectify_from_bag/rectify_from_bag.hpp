#ifndef RECTIFY_FROM_BAG_HPP
#define RECTIFY_FROM_BAG_HPP

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <string>
#include <vector>
#include <iostream>
#include <cmath>
#include <limits>
#include <cstdio>

// ===== ANSI colors =====
static const char* ANSI_ORANGE = "\033[38;5;208m";
static const char* ANSI_RESET  = "\033[0m";

// ===== Feature flags =====
// If true, print detailed debug info about depth decoding, types, min/max, etc.
static const bool ENABLE_DEPTH_DEBUG_PRINTS = false;

// Rectification always uses fixed ZED2i FHD rectified intrinsics for the final image
// (fx=fy=1052.814087, cx=956.342041, cy=553.955444, D=0, R=I, P=[K|0]).

// OpenCV rational_polynomial model parameters
// Radial: k1..k6, tangential: p1,p2
// Distortion vector D = [k1, k2, p1, p2, k3, k4, k5, k6].
struct RationalPolynomialModel {
    double fx, fy, cx, cy;
    double k1, k2, k3, k4, k5, k6;
    double p1, p2;
};

// Choose model based on RGB image topic name
static bool getCustomRationalModelForTopic(const std::string& image_topic,
                                           RationalPolynomialModel& model) {
    // LEFT (FHD intrinsics + LEFT_DISTO rational distortion)
    if (image_topic.find("left") != std::string::npos ||
        image_topic.find("LEFT") != std::string::npos) {
        model.fx = 1071.59;
        model.fy = 1071.52;
        model.cx = 955.79;
        model.cy = 552.776;

        // [LEFT_DISTO] rational_polynomial coefficients
        model.k1 = -0.684597;
        model.k2 =  1.96757;
        model.k3 =  0.17895;
        model.k4 = -0.581677;
        model.k5 =  1.82332;
        model.k6 =  0.333616;
        model.p1 =  0.000330389;
        model.p2 = -8.26626e-05;
        return true;
    }

    // RIGHT (FHD intrinsics + RIGHT_DISTO rational distortion)
    if (image_topic.find("right") != std::string::npos ||
        image_topic.find("RIGHT") != std::string::npos) {
        model.fx = 1070.79;
        model.fy = 1071.06;
        model.cx = 956.55;
        model.cy = 552.334;

        // [RIGHT_DISTO] rational_polynomial coefficients
        model.k1 = -1.6144;
        model.k2 =  2.97561;
        model.k3 =  0.0467966;
        model.k4 = -1.52898;
        model.k5 =  2.82405;
        model.k6 =  0.219175;
        model.p1 = -1.18427e-05;
        model.p2 = -9.7967e-05;
        return true;
    }

    return false;
}


// Build K, D, R, P from OpenCV rational_polynomial model
static void rationalModelToCalibration(const RationalPolynomialModel& m,
                                       cv::Mat& K,
                                       cv::Mat& D,
                                       cv::Mat& R,
                                       cv::Mat& P) {
    // Intrinsic matrix K
    K = cv::Mat::eye(3, 3, CV_64F);
    K.at<double>(0, 0) = m.fx;
    K.at<double>(1, 1) = m.fy;
    K.at<double>(0, 2) = m.cx;
    K.at<double>(1, 2) = m.cy;

    // Distortion: OpenCV rational_polynomial ordering
    // D = [k1, k2, p1, p2, k3, k4, k5, k6]
    D = cv::Mat::zeros(1, 8, CV_64F);
    D.at<double>(0, 0) = m.k1;
    D.at<double>(0, 1) = m.k2;
    D.at<double>(0, 2) = m.p1;
    D.at<double>(0, 3) = m.p2;
    D.at<double>(0, 4) = m.k3;
    D.at<double>(0, 5) = m.k4;
    D.at<double>(0, 6) = m.k5;
    D.at<double>(0, 7) = m.k6;

    // Rectification (identity here)
    R = cv::Mat::eye(3, 3, CV_64F);

    // Projection matrix P = [K | 0]
    P = cv::Mat::zeros(3, 4, CV_64F);
    K.copyTo(P(cv::Rect(0, 0, 3, 3)));
}

// CameraInfo -> K, D, R, P (used when a CameraInfo topic is provided)
static void cameraInfoToCalibration(const sensor_msgs::CameraInfo& info,
                                    cv::Mat& K,
                                    cv::Mat& D,
                                    cv::Mat& R,
                                    cv::Mat& P) {
    // Intrinsic matrix K (3x3)
    K = cv::Mat::zeros(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) {
        K.at<double>(i / 3, i % 3) = info.K[i];
    }

    // Distortion coefficients D (1xN)
    D = cv::Mat::zeros(1, static_cast<int>(info.D.size()), CV_64F);
    for (size_t i = 0; i < info.D.size(); ++i) {
        D.at<double>(0, static_cast<int>(i)) = info.D[i];
    }

    // Rectification matrix R (3x3)
    R = cv::Mat::zeros(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) {
        R.at<double>(i / 3, i % 3) = info.R[i];
    }

    // Projection matrix P (3x4)
    P = cv::Mat::zeros(3, 4, CV_64F);
    for (int i = 0; i < 12; ++i) {
        P.at<double>(i / 4, i % 4) = info.P[i];
    }
}

// Small helper to print intrinsics
static void printIntrinsics(const cv::Mat& K, const std::string& label) {
    std::cout << ANSI_ORANGE;
    if (K.empty() || K.rows != 3 || K.cols != 3) {
        std::cout << label << ": K is empty or not 3x3\n" << ANSI_RESET;
        return;
    }

    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);

    std::cout << label << " K =\n" << K << "\n";
    std::cout << label << " fx = " << fx
              << ", fy = " << fy
              << ", cx = " << cx
              << ", cy = " << cy << "\n";
    std::cout << ANSI_RESET;
}

// helper to print distortion vector D
static void printDistortion(const cv::Mat& D, const std::string& label) {
    std::cout << ANSI_ORANGE;
    if (D.empty() || D.rows != 1) {
        std::cout << label << ": D is empty or not 1xN\n" << ANSI_RESET;
        return;
    }
    std::cout << label << " D = [";
    for (int i = 0; i < D.cols; ++i) {
        double v = D.at<double>(0, i);
        std::cout << v;
        if (i + 1 < D.cols) std::cout << ", ";
    }
    std::cout << "]\n";
    std::cout << ANSI_RESET;
}

// helper to print projection matrix P
static void printProjection(const cv::Mat& P, const std::string& label) {
    std::cout << ANSI_ORANGE;
    if (P.empty() || P.rows != 3 || P.cols != 4) {
        std::cout << label << ": P is empty or not 3x4\n" << ANSI_RESET;
        return;
    }

    double fx = P.at<double>(0, 0);
    double fy = P.at<double>(1, 1);
    double cx = P.at<double>(0, 2);
    double cy = P.at<double>(1, 2);
    double Tx = P.at<double>(0, 3);

    std::cout << label << " P =\n" << P << "\n";
    std::cout << label << " fx' = " << fx
              << ", fy' = " << fy
              << ", cx' = " << cx
              << ", cy' = " << cy
              << ", Tx = " << Tx << "\n";
    std::cout << ANSI_RESET;
}

// helper to print full CameraInfo in orange (RGB only)
static void printCameraInfoMsg(const sensor_msgs::CameraInfo& info,
                               const std::string& label) {
    std::cout << ANSI_ORANGE;
    std::cout << label << ":\n";
    std::cout << "  width=" << info.width << ", height=" << info.height << "\n";
    std::cout << "  distortion_model=" << info.distortion_model << "\n";

    std::cout << "  K (raw) = [";
    for (int i = 0; i < 9; ++i) {
        std::cout << info.K[i];
        if (i + 1 < 9) std::cout << ", ";
    }
    std::cout << "]\n";

    std::cout << "  D (raw) = [";
    for (size_t i = 0; i < info.D.size(); ++i) {
        std::cout << info.D[i];
        if (i + 1 < info.D.size()) std::cout << ", ";
    }
    std::cout << "]\n";

    std::cout << "  R (raw) = [";
    for (int i = 0; i < 9; ++i) {
        std::cout << info.R[i];
        if (i + 1 < 9) std::cout << ", ";
    }
    std::cout << "]\n";

    std::cout << "  P (raw) = [";
    for (int i = 0; i < 12; ++i) {
        std::cout << info.P[i];
        if (i + 1 < 12) std::cout << ", ";
    }
    std::cout << "]\n";
    std::cout << ANSI_RESET;
}

// Build a rectified CameraInfo given optimal intrinsics and projection
static sensor_msgs::CameraInfo makeRectifiedCameraInfo(
    const sensor_msgs::CameraInfo& original,
    const cv::Mat& K_optimal,
    const cv::Mat& P_optimal,
    const cv::Size& image_size)
{
    sensor_msgs::CameraInfo out = original;

    // rectified image size
    out.width  = image_size.width;
    out.height = image_size.height;

    // zero distortion after rectification (5: k1, k2, t1, t2, k3)
    out.D.assign(5, 0.0);
    out.distortion_model = "plumb_bob";

    // R = identity (R is boost::array<double, 9>, so assign takes only a value)
    out.R.assign(0.0);                 // zero all entries
    out.R[0] = out.R[4] = out.R[8] = 1.0;

    // K from K_optimal
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out.K[r * 3 + c] = K_optimal.at<double>(r, c);
        }
    }

    // P from P_optimal
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 4; ++c) {
            out.P[r * 4 + c] = P_optimal.at<double>(r, c);
        }
    }

    return out;
}


// Topic comparison helper (handle leading / inconsistencies)
static bool topicMatches(const std::string& t, const std::string& target) {
    if (target.empty()) return false;
    if (t == target) return true;
    if (target[0] == '/' && t == target.substr(1)) return true;
    if (!t.empty() && t[0] == '/' && t.substr(1) == target) return true;
    return false;
}

// Helper to print cv::Mat basic info
static void printMatInfo(const cv::Mat& m, const std::string& label) {
    std::cout << label << ": rows=" << m.rows
              << ", cols=" << m.cols
              << ", channels=" << m.channels()
              << ", depth=" << m.depth()
              << ", type=" << m.type()
              << ", step=" << m.step
              << ", isContinuous=" << (m.isContinuous() ? "true" : "false")
              << "\n";
}

// Compute min/max (ignoring NaN/inf) for CV_32F single-channel
static bool computeDepthMinMax(const cv::Mat& depth_f,
                               double& minVal,
                               double& maxVal,
                               size_t& nanCount,
                               size_t& infCount) {
    if (depth_f.empty() || depth_f.type() != CV_32FC1) {
        if (ENABLE_DEPTH_DEBUG_PRINTS) {
            std::cerr << "[Depth] computeDepthMinMax: invalid mat (empty or not CV_32FC1)\n";
        }
        return false;
    }

    minVal = std::numeric_limits<double>::infinity();
    maxVal = -std::numeric_limits<double>::infinity();
    nanCount = 0;
    infCount = 0;

    const int rows = depth_f.rows;
    const int cols = depth_f.cols;

    for (int r = 0; r < rows; ++r) {
        const float* rowPtr = depth_f.ptr<float>(r);
        for (int c = 0; c < cols; ++c) {
            float v = rowPtr[c];
            if (std::isnan(v)) {
                ++nanCount;
                continue;
            }
            if (!std::isfinite(v)) {
                ++infCount;
                continue;
            }
            if (v < minVal) minVal = v;
            if (v > maxVal) maxVal = v;
        }
    }

    if (!std::isfinite(minVal) || !std::isfinite(maxVal)) {
        if (ENABLE_DEPTH_DEBUG_PRINTS) {
            std::cerr << "[Depth] computeDepthMinMax: no finite values found\n";
        }
        return false;
    }

    if (minVal == maxVal) {
        if (ENABLE_DEPTH_DEBUG_PRINTS) {
            std::cerr << "[Depth] computeDepthMinMax: min==max (" << minVal
                      << "); cannot normalize\n";
        }
        return false;
    }

    return true;
}

// Map CV_32F depth to CV_8U using given min/max, ignoring NaN/inf
static void depthToU8WithMinMax(const cv::Mat& depth_f,
                                cv::Mat& depth_u8,
                                double minVal,
                                double maxVal) {
    depth_u8.create(depth_f.size(), CV_8U);

    const int rows = depth_f.rows;
    const int cols = depth_f.cols;
    const double invRange = 255.0 / (maxVal - minVal);

    for (int r = 0; r < rows; ++r) {
        const float* rowPtr = depth_f.ptr<float>(r);
        uchar* outRow = depth_u8.ptr<uchar>(r);
        for (int c = 0; c < cols; ++c) {
            float v = rowPtr[c];
            if (!std::isfinite(v)) {
                outRow[c] = 0;
                continue;
            }
            double x = (static_cast<double>(v) - minVal) * invRange;
            if (x < 0.0) x = 0.0;
            if (x > 255.0) x = 255.0;
            outRow[c] = static_cast<uchar>(x + 0.5);
        }
    }
}

// Decode depth, resize to target_size, and produce a single colorized depth image.
// No geometric rectification or remap is applied to depth.
static bool decodeAndColorizeDepth(const rosbag::MessageInstance& depth_msg,
                                   const cv::Size& target_size,
                                   cv::Mat& depth_color)
{
    cv::Mat depth_raw;

    if (ENABLE_DEPTH_DEBUG_PRINTS) {
        std::cout << "[Depth] decodeAndColorizeDepth: target_size="
                  << target_size.width << "x" << target_size.height << "\n";
    }

    // Try raw Image first
    if (auto d_raw = depth_msg.instantiate<sensor_msgs::Image>()) {
        if (ENABLE_DEPTH_DEBUG_PRINTS) {
            std::cout << "[Depth] Message is sensor_msgs::Image\n";
        }
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(d_raw);
        } catch (const cv_bridge::Exception& e) {
            if (ENABLE_DEPTH_DEBUG_PRINTS) {
                std::cerr << "cv_bridge exception (depth Image): " << e.what() << "\n";
            }
            return false;
        }
        // Deep copy to own the data (avoid dangling pointer when cv_ptr/d_raw go out of scope)
        depth_raw = cv_ptr->image.clone();
    } else if (auto d_cmp = depth_msg.instantiate<sensor_msgs::CompressedImage>()) {
        if (ENABLE_DEPTH_DEBUG_PRINTS) {
            std::cout << "[Depth] Message is sensor_msgs::CompressedImage\n";
        }
        const std::vector<uint8_t>& data = d_cmp->data;
        if (data.empty()) {
            if (ENABLE_DEPTH_DEBUG_PRINTS) {
                std::cerr << "Compressed depth has empty data; skipping\n";
            }
            return false;
        }
        cv::Mat buf(1, static_cast<int>(data.size()), CV_8UC1,
                    const_cast<uint8_t*>(data.data()));
        depth_raw = cv::imdecode(buf, cv::IMREAD_UNCHANGED);
        if (depth_raw.empty()) {
            if (ENABLE_DEPTH_DEBUG_PRINTS) {
                std::cerr << "imdecode failed for depth; skipping\n";
            }
            return false;
        }
    } else {
        if (ENABLE_DEPTH_DEBUG_PRINTS) {
            std::cerr << "Depth message is not Image or CompressedImage; skipping\n";
        }
        return false;
    }

    if (depth_raw.empty()) {
        if (ENABLE_DEPTH_DEBUG_PRINTS) {
            std::cerr << "[Depth] Decoded depth is empty; skipping\n";
        }
        return false;
    }

    if (ENABLE_DEPTH_DEBUG_PRINTS) {
        printMatInfo(depth_raw, "[Depth] depth_raw");
    }

    // Infer type and convert to float
    cv::Mat depth_f;

    int depthDepth = depth_raw.depth();
    int depthChannels = depth_raw.channels();

    if (ENABLE_DEPTH_DEBUG_PRINTS) {
        std::cout << "[Depth] depth_raw depth=" << depthDepth
                  << ", channels=" << depthChannels << "\n";
    }

    if (depthDepth == CV_32F && depthChannels == 1) {
        if (ENABLE_DEPTH_DEBUG_PRINTS) {
            std::cout << "[Depth] Interpreting as CV_32F depth; using as-is (no convertTo)\n";
        }
        depth_f = depth_raw;  // already deep-owned by clone above
    } else if (depthDepth == CV_16U && depthChannels == 1) {
        if (ENABLE_DEPTH_DEBUG_PRINTS) {
            std::cout << "[Depth] Interpreting as CV_16U depth; converting to CV_32F\n";
        }
        depth_raw.convertTo(depth_f, CV_32F);
    } else if (depthDepth == CV_8U && depthChannels == 1) {
        if (ENABLE_DEPTH_DEBUG_PRINTS) {
            std::cout << "[Depth] Interpreting as CV_8U depth; converting to CV_32F\n";
        }
        depth_raw.convertTo(depth_f, CV_32F);
    } else {
        if (ENABLE_DEPTH_DEBUG_PRINTS) {
            std::cout << "[Depth] Unexpected depth type (depth=" << depthDepth
                      << ", channels=" << depthChannels
                      << "); converting to CV_32F anyway\n";
        }
        depth_raw.convertTo(depth_f, CV_32F);
    }

    if (depth_f.empty()) {
        if (ENABLE_DEPTH_DEBUG_PRINTS) {
            std::cerr << "[Depth] depth_f is empty right after conversion; skipping\n";
        }
        return false;
    }

    if (ENABLE_DEPTH_DEBUG_PRINTS) {
        printMatInfo(depth_f, "[Depth] depth_f after convert / alias");
    }

    // Ensure CV_32FC1
    if (depth_f.type() != CV_32FC1) {
        if (ENABLE_DEPTH_DEBUG_PRINTS) {
            std::cout << "[Depth] depth_f is not CV_32FC1 after conversion; enforcing CV_32FC1\n";
        }
        depth_f.convertTo(depth_f, CV_32F);
    }

    if (ENABLE_DEPTH_DEBUG_PRINTS) {
        printMatInfo(depth_f, "[Depth] depth_f after type enforcement");
    }

    // Pure resize (no rectification / remap); only to match RGB resolution
    // if (depth_f.size() != target_size &&
    //     target_size.width > 0 && target_size.height > 0) {
    //     if (ENABLE_DEPTH_DEBUG_PRINTS) {
    //         std::cout << "[Depth] Resizing depth_f from "
    //                   << depth_f.cols << "x" << depth_f.rows
    //                   << " to " << target_size.width << "x" << target_size.height
    //                   << " (INTER_NEAREST)\n";
    //     }
    //     cv::resize(depth_f, depth_f, target_size, 0, 0, cv::INTER_NEAREST);
    // }

    if (depth_f.empty()) {
        if (ENABLE_DEPTH_DEBUG_PRINTS) {
            std::cerr << "[Depth] Depth after resize is empty; skipping\n";
        }
        return false;
    }

    if (ENABLE_DEPTH_DEBUG_PRINTS) {
        printMatInfo(depth_f, "[Depth] depth_f after resize");
    }

    // Compute min/max once
    double minVal = 0.0;
    double maxVal = 0.0;
    size_t nanCount = 0;
    size_t infCount = 0;

    if (!computeDepthMinMax(depth_f, minVal, maxVal, nanCount, infCount)) {
        if (ENABLE_DEPTH_DEBUG_PRINTS) {
            std::cerr << "[Depth] Failed to compute min/max; nanCount=" << nanCount
                      << ", infCount=" << infCount << "\n";
        }
        return false;
    }

    if (ENABLE_DEPTH_DEBUG_PRINTS) {
        std::cout << "[Depth] minVal=" << minVal
                  << ", maxVal=" << maxVal
                  << ", nanCount=" << nanCount
                  << ", infCount=" << infCount << "\n";
    }

    // Colorized depth (single output, in whatever geometry the depth is already in)
    cv::Mat depth_u8;
    if (ENABLE_DEPTH_DEBUG_PRINTS) {
        std::cout << "[Depth] Mapping depth_f -> depth_u8 using custom min/max\n";
    }
    depthToU8WithMinMax(depth_f, depth_u8, minVal, maxVal);

    if (ENABLE_DEPTH_DEBUG_PRINTS) {
        printMatInfo(depth_u8, "[Depth] depth_u8");
    }

    cv::applyColorMap(depth_u8, depth_color, cv::COLORMAP_JET);

    if (ENABLE_DEPTH_DEBUG_PRINTS) {
        printMatInfo(depth_color, "[Depth] depth_color");
    }

    return !depth_color.empty();
}

// ===== Zoom viewer state & helpers =====
static cv::Mat g_current_canvas;
static bool g_has_canvas = false;
static bool g_zoom_enabled = false;
static double g_zoom_factor = 1.0;
static cv::Point2d g_zoom_center(0.0, 0.0);
static bool g_zoom_center_initialized = false;
static bool g_paused_global = false;

static void updateZoomedDisplay(const std::string& window_name)
{
    if (!g_has_canvas || g_current_canvas.empty())
        return;

    const cv::Mat& src = g_current_canvas;
    cv::Mat display;

    if (!g_zoom_enabled || g_zoom_factor <= 1.0) {
        display = src;
    } else {
        int W = src.cols;
        int H = src.rows;

        double z = std::max(1.0, std::min(g_zoom_factor, 20.0));

        if (!g_zoom_center_initialized ||
            g_zoom_center.x < 0 || g_zoom_center.x >= W ||
            g_zoom_center.y < 0 || g_zoom_center.y >= H) {
            g_zoom_center = cv::Point2d(W * 0.5, H * 0.5);
            g_zoom_center_initialized = true;
        }

        int roi_w = std::max(1, static_cast<int>(std::round(W / z)));
        int roi_h = std::max(1, static_cast<int>(std::round(H / z)));

        double cx = g_zoom_center.x;
        double cy = g_zoom_center.y;

        int x0 = static_cast<int>(std::round(cx - roi_w * 0.5));
        int y0 = static_cast<int>(std::round(cy - roi_h * 0.5));

        if (x0 < 0) x0 = 0;
        if (y0 < 0) y0 = 0;
        if (x0 + roi_w > W) x0 = W - roi_w;
        if (y0 + roi_h > H) y0 = H - roi_h;

        cv::Rect roi(x0, y0, roi_w, roi_h);
        cv::Mat cropped = src(roi);

        if (roi_w == W && roi_h == H) {
            display = cropped;
        } else {
            cv::resize(cropped, display, src.size(), 0, 0, cv::INTER_LINEAR);
        }
    }

    if (g_zoom_enabled && g_zoom_factor > 1.0) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "Zoom x%.2f", g_zoom_factor);
        cv::putText(display,
                    buf,
                    cv::Point(30, g_current_canvas.rows - 30),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.8,
                    CV_RGB(255, 255, 255),
                    2,
                    cv::LINE_AA);
    }

    cv::imshow(window_name, display);
}

static void onMouse(int event, int x, int y, int flags, void* userdata)
{
    (void)flags;
    (void)userdata;

    if (!g_paused_global)
        return;
    if (!g_zoom_enabled)
        return;
    if (!g_has_canvas || g_current_canvas.empty())
        return;
    if (event != cv::EVENT_LBUTTONDOWN && event != cv::EVENT_LBUTTONDBLCLK)
        return;

    const cv::Mat& src = g_current_canvas;
    int W = src.cols;
    int H = src.rows;

    double z = std::max(1.0, std::min(g_zoom_factor, 20.0));

    int roi_w = std::max(1, static_cast<int>(std::round(W / z)));
    int roi_h = std::max(1, static_cast<int>(std::round(H / z)));

    double cx = g_zoom_center.x;
    double cy = g_zoom_center.y;

    int x0 = static_cast<int>(std::round(cx - roi_w * 0.5));
    int y0 = static_cast<int>(std::round(cy - roi_h * 0.5));

    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x0 + roi_w > W) x0 = W - roi_w;
    if (y0 + roi_h > H) y0 = H - roi_h;

    double base_x = x0 + static_cast<double>(x) / z;
    double base_y = y0 + static_cast<double>(y) / z;

    if (base_x < 0.0) base_x = 0.0;
    if (base_y < 0.0) base_y = 0.0;
    if (base_x >= W) base_x = W - 1.0;
    if (base_y >= H) base_y = H - 1.0;

    g_zoom_center = cv::Point2d(base_x, base_y);
    g_zoom_center_initialized = true;

    updateZoomedDisplay("distorted vs rectified with depth");
}

#endif // RECTIFY_FROM_BAG_HPP
