#include <rectify_from_bag/rectify_from_bag.hpp>
#include <rectify_from_bag/turbojpeg_utils.hpp>

#include <filesystem>
#include <sstream>

static bool requireParamNonEmpty(const ros::NodeHandle& pnh,
                                 const std::string& key,
                                 std::string& out)
{
    if (!pnh.getParam(key, out) || out.empty()) {
        std::cerr << "Missing/empty required param: ~" << key << "\n";
        return false;
    }
    return true;
}

static bool getStampFromMsg(const rosbag::MessageInstance& mi, ros::Time& stamp_out)
{
    if (auto m = mi.instantiate<sensor_msgs::Image>()) {
        stamp_out = m->header.stamp;
        return true;
    }
    if (auto m = mi.instantiate<sensor_msgs::CompressedImage>()) {
        stamp_out = m->header.stamp;
        return true;
    }
    return false;
}

static void ensureSameSize(const cv::Mat& ref, cv::Mat& img, int interp)
{
    if (ref.empty() || img.empty()) return;
    if (img.size() == ref.size()) return;
    cv::resize(img, img, ref.size(), 0, 0, interp);
}

static bool ensureDir(const std::string& dir)
{
    if (dir.empty()) return false;
    try {
        std::filesystem::path p(dir);
        if (std::filesystem::exists(p)) return std::filesystem::is_directory(p);
        return std::filesystem::create_directories(p);
    } catch (...) {
        return false;
    }
}

static std::string makePngPath(const std::string& dir, const ros::Time& stamp, const std::string& prefix)
{
    std::ostringstream oss;
    oss << prefix << "_" << stamp.sec << "_" << std::setw(9) << std::setfill('0') << stamp.nsec << ".png";
    std::filesystem::path out = std::filesystem::path(dir) / oss.str();
    return out.string();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rectify_from_bag");
    ros::NodeHandle pnh("~");

    // Required
    std::string rgb_bag_path, rgb_image_topic;
    if (!requireParamNonEmpty(pnh, "rgb_bag", rgb_bag_path)) return 1;
    if (!requireParamNonEmpty(pnh, "rgb_image_topic", rgb_image_topic)) return 1;

    // Optional CameraInfo
    std::string rgb_info_topic;
    pnh.param<std::string>("rgb_info_topic", rgb_info_topic, std::string(""));
    const bool have_camera_info_topic = !rgb_info_topic.empty();

    // Optional depth stream
    bool use_depth = false;
    pnh.param("use_depth", use_depth, false);

    std::string depth_bag_path, depth_image_topic, depth_info_topic;
    if (use_depth) {
        if (!requireParamNonEmpty(pnh, "depth_bag", depth_bag_path)) return 1;
        if (!requireParamNonEmpty(pnh, "depth_image_topic", depth_image_topic)) return 1;
        pnh.param<std::string>("depth_info_topic", depth_info_topic, std::string(""));
    }

    // Optional confidence stream
    bool use_confidence = false;
    pnh.param("use_confidence", use_confidence, false);

    std::string confidence_bag_path, confidence_image_topic;
    pnh.param<std::string>("confidence_bag", confidence_bag_path, std::string(""));
    pnh.param<std::string>("confidence_image_topic",
                           confidence_image_topic,
                           std::string("/boxi/zed2i/confidence/image_raw/compressed"));

    // Optional saving of processed confidence images
    std::string confidence_save_dir;
    bool confidence_save_colormap = true;
    pnh.param<std::string>("confidence_save_dir", confidence_save_dir, std::string(""));
    pnh.param("confidence_save_colormap", confidence_save_colormap, true);

    bool enable_confidence_saving = false;
    if (!confidence_save_dir.empty()) {
        if (ensureDir(confidence_save_dir)) {
            enable_confidence_saving = true;
            std::cout << ANSI_ORANGE
                      << "Confidence saving enabled: " << confidence_save_dir << "\n"
                      << ANSI_RESET;
        } else {
            std::cerr << "[WARN] Could not create/access confidence_save_dir='"
                      << confidence_save_dir << "'. Disabling confidence saving.\n";
        }
    }

    // If user wants confidence but didn't specify a bag, default to depth_bag if available
    if (use_confidence && confidence_bag_path.empty()) {
        if (use_depth && !depth_bag_path.empty()) {
            confidence_bag_path = depth_bag_path;
            std::cout << ANSI_ORANGE
                      << "confidence_bag not set; defaulting to depth_bag: "
                      << confidence_bag_path << "\n"
                      << ANSI_RESET;
        } else {
            std::cerr << "use_confidence=true but ~confidence_bag is empty and no depth_bag available.\n";
            return 1;
        }
    }

    const bool rgb_topic_is_rectified =
        (rgb_image_topic.find("rect") != std::string::npos);

    if (rgb_topic_is_rectified) {
        std::cout << ANSI_ORANGE
                  << "RGB image topic '" << rgb_image_topic
                  << "' contains 'rect'; assuming images are already rectified. "
                  << "No additional cv::remap() rectification will be applied.\n"
                  << ANSI_RESET;
    }

    // Calibration and rectification maps.
    cv::Mat K, D, R, P;
    cv::Mat K_optimal;
    cv::Mat map1, map2;
    bool maps_initialized = false;
    cv::Size map_size;
    bool have_calibration = false;

    // Store original and rectified CameraInfo
    sensor_msgs::CameraInfo original_rgb_cam_info;
    bool have_original_rgb_cam_info = false;
    sensor_msgs::CameraInfo rectified_rgb_cam_info;
    bool have_rectified_rgb_cam_info = false;

    // Fallback: custom rational_polynomial model if NO CameraInfo topic is provided
    RationalPolynomialModel custom_model{};
    bool have_custom_model = false;
    if (!have_camera_info_topic) {
        have_custom_model = getCustomRationalModelForTopic(rgb_image_topic, custom_model);
        if (have_custom_model) {
            rationalModelToCalibration(custom_model, K, D, R, P);
            have_calibration = true;
            std::cout << ANSI_ORANGE
                      << "Using custom rational_polynomial calibration for topic '"
                      << rgb_image_topic << "' for distortion correction (no CameraInfo topic)\n"
                      << ANSI_RESET;
            printIntrinsics(K, "Initial (custom) intrinsics (raw K)");
            printDistortion(D, "Initial (custom) distortion (raw D)");
            printProjection(P, "Initial (custom) projection (raw P)");
        } else {
            std::cerr << "No CameraInfo topic provided and no custom model match "
                      << "for image topic '" << rgb_image_topic << "'.\n";
        }
    }

    // Open RGB bag
    rosbag::Bag rgb_bag;
    try {
        rgb_bag.open(rgb_bag_path, rosbag::bagmode::Read);
    } catch (const rosbag::BagException& e) {
        std::cerr << "Failed to open RGB bag: " << e.what() << "\n";
        return 1;
    }

    // Depth bag/view/iter
    rosbag::Bag depth_bag;
    rosbag::View depth_view;
    rosbag::View::const_iterator depth_it;
    rosbag::View::const_iterator depth_end;

    if (use_depth) {
        try {
            depth_bag.open(depth_bag_path, rosbag::bagmode::Read);
        } catch (const rosbag::BagException& e) {
            std::cerr << "Failed to open depth bag: " << e.what() << "\n";
            return 1;
        }

        depth_view.addQuery(depth_bag, rosbag::TopicQuery({depth_image_topic}));
        depth_it  = depth_view.begin();
        depth_end = depth_view.end();
    }

    // Confidence bag/view/iter (may reuse depth_bag if same path)
    bool confidence_uses_depth_bag = false;
    rosbag::Bag confidence_bag;
    rosbag::View confidence_view;
    rosbag::View::const_iterator conf_it;
    rosbag::View::const_iterator conf_end;

    if (use_confidence) {
        confidence_uses_depth_bag = (use_depth && (confidence_bag_path == depth_bag_path));
        if (confidence_uses_depth_bag) {
            confidence_view.addQuery(depth_bag, rosbag::TopicQuery({confidence_image_topic}));
        } else {
            try {
                confidence_bag.open(confidence_bag_path, rosbag::bagmode::Read);
            } catch (const rosbag::BagException& e) {
                std::cerr << "Failed to open confidence bag: " << e.what() << "\n";
                return 1;
            }
            confidence_view.addQuery(confidence_bag, rosbag::TopicQuery({confidence_image_topic}));
        }

        conf_it  = confidence_view.begin();
        conf_end = confidence_view.end();
    }

    // RGB view over image (+ CameraInfo if provided)
    std::vector<std::string> rgb_topics;
    rgb_topics.push_back(rgb_image_topic);
    if (have_camera_info_topic) rgb_topics.push_back(rgb_info_topic);
    rosbag::View rgb_view(rgb_bag, rosbag::TopicQuery(rgb_topics));

    bool had_messages  = false;
    size_t num_infos   = 0;
    size_t num_images  = 0;

    cv::namedWindow("distorted vs rectified with depth/confidence", cv::WINDOW_NORMAL);
    cv::setMouseCallback("distorted vs rectified with depth/confidence", onMouse, nullptr);

    std::cout << ANSI_ORANGE
              << "Controls: p=Pause/Resume, q/ESC=Quit, z=Toggle zoom (while paused),\n"
              << "          +/-=Zoom in/out, WASD=Pan (while paused & zoom enabled),\n"
              << "          Left-click=move zoom center (while paused & zoom enabled)\n"
              << ANSI_RESET;

    bool paused = false;
    bool window_initialized = false;

    for (const rosbag::MessageInstance& m : rgb_view) {
        had_messages = true;
        const std::string& topic = m.getTopic();

        // RGB CameraInfo
        if (have_camera_info_topic && topicMatches(topic, rgb_info_topic)) {
            auto info = m.instantiate<sensor_msgs::CameraInfo>();
            if (!info) {
                std::cerr << "Failed to deserialize CameraInfo on topic "
                          << topic << "\n";
                continue;
            }

            ++num_infos;

            printCameraInfoMsg(*info, "RGB CameraInfo message (raw values)");
            original_rgb_cam_info = *info;
            have_original_rgb_cam_info = true;

            cameraInfoToCalibration(*info, K, D, R, P);
            have_calibration = true;
            maps_initialized = false;

            printIntrinsics(K, "Intrinsics from CameraInfo (raw K)");
            printDistortion(D, "Distortion from CameraInfo (raw D)");
            printProjection(P, "Projection from CameraInfo (raw P)");
            continue;
        }

        // RGB Image
        if (!topicMatches(topic, rgb_image_topic)) continue;

        cv::Mat distorted;
        ros::Time rgb_stamp;

        if (auto img_raw = m.instantiate<sensor_msgs::Image>()) {
            rgb_stamp = img_raw->header.stamp;

            cv_bridge::CvImageConstPtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvShare(img_raw);
            } catch (const cv_bridge::Exception& e) {
                std::cerr << "cv_bridge exception (RGB Image) on "
                          << topic << ": " << e.what() << "\n";
                continue;
            }
            distorted = cv_ptr->image.clone();
        } else if (auto img_cmp = m.instantiate<sensor_msgs::CompressedImage>()) {
            rgb_stamp = img_cmp->header.stamp;

            if (!rectify_from_bag::turbojpeg_utils::decodeCompressedBGR(*img_cmp, distorted)) {
                std::cerr << "JPEG decode failed for RGB on topic "
                          << topic << "; skipping\n";
                continue;
            }
        } else {
            std::cerr << "Message on topic " << topic
                      << " is not sensor_msgs/Image or sensor_msgs/CompressedImage; skipping\n";
            continue;
        }

        if (!have_calibration) {
            std::cerr << "RGB image received but no calibration available; skipping\n";
            continue;
        }
        if (distorted.empty()) {
            std::cerr << "Decoded RGB image is empty; skipping\n";
            continue;
        }

        ++num_images;

        if (!maps_initialized || map_size != distorted.size()) {
            map_size = distorted.size();

            cv::Mat K_raw = K.clone();
            cv::Mat D_raw = D.clone();
            cv::Mat P_raw = P.clone();

            if (rgb_topic_is_rectified) {
                std::cout << ANSI_ORANGE
                          << "RGB image topic '" << rgb_image_topic
                          << "' is treated as already rectified; "
                          << "skipping computation of undistort/rectify maps.\n"
                          << ANSI_RESET;

                K_optimal = K_raw.clone();
                if (!P_raw.empty()) {
                    P = P_raw.clone();
                } else if (!K_optimal.empty()) {
                    P = cv::Mat::zeros(3, 4, CV_64F);
                    K_optimal.copyTo(P(cv::Rect(0, 0, 3, 3)));
                }

                printIntrinsics(K_raw, "RGB intrinsics (already rectified, K_raw)");
                printDistortion(D_raw, "RGB distortion (already rectified, D_raw)");
                printProjection(P, "RGB projection (already rectified, P)");

                if (have_original_rgb_cam_info) {
                    rectified_rgb_cam_info =
                        makeRectifiedCameraInfo(original_rgb_cam_info,
                                                K_optimal,
                                                P,
                                                map_size);
                    have_rectified_rgb_cam_info = true;
                    printCameraInfoMsg(rectified_rgb_cam_info,
                                       "Rectified RGB CameraInfo (already-rectified topic)");
                }

                map1.release();
                map2.release();
                maps_initialized = false;
            } else {
                std::cout << ANSI_ORANGE
                          << "Rectification: using current RGB K,D for distortion correction\n"
                          << "and FORCING ZED2i FHD rectified intrinsics "
                          << "(fx=fy=1052.814087, cx=956.342041, cy=553.955444).\n"
                          << ANSI_RESET;

                if (map_size.width != 1920 || map_size.height != 1080) {
                    std::cerr << "[WARN] Forcing ZED2i FHD intrinsics on "
                              << map_size.width << "x" << map_size.height
                              << " image. Expected 1920x1080.\n";
                }

                cv::Mat K_forced = cv::Mat::zeros(3, 3, CV_64F);
                K_forced.at<double>(0, 0) = 1052.814087;
                K_forced.at<double>(1, 1) = 1052.814087;
                K_forced.at<double>(0, 2) = 956.342041;
                K_forced.at<double>(1, 2) = 553.955444;
                K_forced.at<double>(2, 2) = 1.0;

                cv::initUndistortRectifyMap(
                    K_raw, D_raw, R, K_forced,
                    map_size, CV_32FC1, map1, map2
                );

                K_optimal = K_forced.clone();
                P = cv::Mat::zeros(3, 4, CV_64F);
                K_optimal.copyTo(P(cv::Rect(0, 0, 3, 3)));

                printIntrinsics(K_raw, "RGB intrinsics BEFORE rectification (K_raw)");
                printDistortion(D_raw, "RGB distortion BEFORE rectification (D_raw)");
                printProjection(P_raw, "RGB projection BEFORE rectification (P_raw)");

                printIntrinsics(K_optimal, "RGB intrinsics AFTER rectification (K_optimal)");
                if (!D_raw.empty()) {
                    cv::Mat D_rectified = cv::Mat::zeros(D_raw.rows, D_raw.cols, D_raw.type());
                    printDistortion(D_rectified, "RGB distortion AFTER rectification (D_rectified = 0)");
                }
                printProjection(P, "RGB projection AFTER rectification (P_optimal)");

                maps_initialized = true;

                if (have_original_rgb_cam_info) {
                    rectified_rgb_cam_info =
                        makeRectifiedCameraInfo(original_rgb_cam_info,
                                                K_optimal,
                                                P,
                                                map_size);
                    have_rectified_rgb_cam_info = true;
                    printCameraInfoMsg(rectified_rgb_cam_info,
                                       "Rectified RGB CameraInfo (after rectification)");
                }
            }
        }

        // Rectify RGB (or pass-through)
        cv::Mat rectified;
        if (rgb_topic_is_rectified) {
            rectified = distorted.clone();
        } else {
            cv::remap(distorted, rectified, map1, map2,
                      cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        }
        if (rectified.empty()) continue;

        // ---- Depth: exact timestamp match, consume once ----
        cv::Mat depth_color;
        bool have_depth_for_frame = false;

        if (use_depth && depth_view.size() > 0) {
            while (depth_it != depth_end) {
                ros::Time depth_stamp;
                if (!getStampFromMsg(*depth_it, depth_stamp)) { ++depth_it; continue; }

                if (depth_stamp < rgb_stamp) {
                    ++depth_it;
                    continue;
                } else if (depth_stamp == rgb_stamp) {
                    if (decodeAndColorizeDepth(*depth_it, rectified.size(), depth_color)) {
                        ensureSameSize(rectified, depth_color, cv::INTER_NEAREST);
                        have_depth_for_frame = !depth_color.empty();
                    }
                    ++depth_it; // consume exactly once
                    break;
                } else {
                    break; // depth ahead; keep for future RGB
                }
            }
        }

        // ---- Confidence: exact timestamp match, consume once ----
        cv::Mat conf_color;
        bool have_conf_for_frame = false;

        if (use_confidence && confidence_view.size() > 0) {
            while (conf_it != conf_end) {
                ros::Time conf_stamp;
                if (!getStampFromMsg(*conf_it, conf_stamp)) { ++conf_it; continue; }

                if (conf_stamp < rgb_stamp) {
                    ++conf_it;
                    continue;
                } else if (conf_stamp == rgb_stamp) {
                    // Reuse same processing pipeline as depth (decode -> normalize -> colormap)
                    if (decodeAndColorizeDepth(*conf_it, rectified.size(), conf_color)) {
                        ensureSameSize(rectified, conf_color, cv::INTER_NEAREST);
                        have_conf_for_frame = !conf_color.empty();

                        if (enable_confidence_saving && confidence_save_colormap && have_conf_for_frame) {
                            const std::string path = makePngPath(confidence_save_dir, conf_stamp, "confidence_color");
                            try {
                                cv::imwrite(path, conf_color);
                            } catch (...) {
                                std::cerr << "[WARN] Failed to write confidence image: " << path << "\n";
                            }
                        }
                    }
                    ++conf_it; // consume exactly once
                    break;
                } else {
                    break; // confidence ahead; keep for future RGB
                }
            }
        }

        // ---- Build displays ----
        cv::Mat distorted_depth = distorted.clone();
        cv::Mat rectified_depth = rectified.clone();
        if (have_depth_for_frame && !depth_color.empty()) {
            const double alpha_m = 0.6;
            const double alpha_i = 0.4;
            cv::addWeighted(depth_color, alpha_m, rectified, alpha_i, 0.0, rectified_depth);
            cv::addWeighted(depth_color, alpha_m, distorted, alpha_i, 0.0, distorted_depth);
        }

        cv::Mat distorted_conf = distorted.clone();
        cv::Mat rectified_conf = rectified.clone();
        if (have_conf_for_frame && !conf_color.empty()) {
            const double alpha_m = 0.6;
            const double alpha_i = 0.4;
            cv::addWeighted(conf_color, alpha_m, rectified, alpha_i, 0.0, rectified_conf);
            cv::addWeighted(conf_color, alpha_m, distorted, alpha_i, 0.0, distorted_conf);
        }

        // ---- Compose 2x2 canvas:
        // top row: depth overlays (distorted | rectified)
        // bottom row: confidence overlays (distorted | rectified)
        const int W = distorted.cols;
        const int H = distorted.rows;

        cv::Mat canvas(2 * H, 2 * W, distorted.type(), cv::Scalar(0,0,0));

        distorted_depth.copyTo(canvas(cv::Rect(0,     0, W, H)));
        rectified_depth.copyTo(canvas(cv::Rect(W,     0, W, H)));
        distorted_conf.copyTo(canvas(cv::Rect(0,     H, W, H)));
        rectified_conf.copyTo(canvas(cv::Rect(W,     H, W, H)));

        cv::putText(canvas, "Distorted + depth",  cv::Point(30, 40),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, CV_RGB(255,255,255), 2, cv::LINE_AA);
        cv::putText(canvas, "Rectified + depth",  cv::Point(W + 30, 40),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, CV_RGB(255,255,255), 2, cv::LINE_AA);
        cv::putText(canvas, "Distorted + confidence", cv::Point(30, H + 40),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, CV_RGB(255,255,255), 2, cv::LINE_AA);
        cv::putText(canvas, "Rectified + confidence", cv::Point(W + 30, H + 40),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, CV_RGB(255,255,255), 2, cv::LINE_AA);

        // Store canvas for zoom
        g_current_canvas = canvas.clone();
        g_has_canvas = true;

        if (!g_zoom_center_initialized) {
            g_zoom_center = cv::Point2d(canvas.cols * 0.5, canvas.rows * 0.5);
            g_zoom_center_initialized = true;
        } else {
            if (g_zoom_center.x < 0) g_zoom_center.x = 0;
            if (g_zoom_center.y < 0) g_zoom_center.y = 0;
            if (g_zoom_center.x >= canvas.cols) g_zoom_center.x = canvas.cols - 1;
            if (g_zoom_center.y >= canvas.rows) g_zoom_center.y = canvas.rows - 1;
        }

        if (!window_initialized) {
            cv::resizeWindow("distorted vs rectified with depth/confidence", canvas.cols, canvas.rows);
            window_initialized = true;
        }

        updateZoomedDisplay("distorted vs rectified with depth/confidence");

        // ---- Controls loop ----
        bool quit = false;
        g_paused_global = paused;

        while (true) {
            int key = cv::waitKey(30);

            if (key == 'p') {
                paused = !paused;
                g_paused_global = paused;
                std::cout << (paused ? "Paused\n" : "Resumed\n");
            } else if (key == 27 || key == 'q') {
                quit = true;
                break;
            }

            if (paused) {
                bool updated = false;

                if (key == 'z' || key == 'Z') {
                    g_zoom_enabled = !g_zoom_enabled;
                    if (!g_zoom_enabled) {
                        g_zoom_factor = 1.0;
                    } else {
                        if (!g_has_canvas || g_current_canvas.empty()) {
                            g_zoom_enabled = false;
                        }
                    }
                    updated = true;
                } else if (g_zoom_enabled) {
                    if (key == '+' || key == '=') {
                        g_zoom_factor *= 1.25;
                        if (g_zoom_factor > 20.0) g_zoom_factor = 20.0;
                        updated = true;
                    } else if (key == '-' || key == '_') {
                        g_zoom_factor /= 1.25;
                        if (g_zoom_factor < 1.0) g_zoom_factor = 1.0;
                        updated = true;
                    } else if (key == 'a' || key == 'A' ||
                               key == 'd' || key == 'D' ||
                               key == 'w' || key == 'W' ||
                               key == 's' || key == 'S') {
                        double step = 50.0;
                        if (key == 'a' || key == 'A') g_zoom_center.x -= step;
                        if (key == 'd' || key == 'D') g_zoom_center.x += step;
                        if (key == 'w' || key == 'W') g_zoom_center.y -= step;
                        if (key == 's' || key == 'S') g_zoom_center.y += step;

                        if (g_has_canvas && !g_current_canvas.empty()) {
                            int cW = g_current_canvas.cols;
                            int cH = g_current_canvas.rows;
                            if (g_zoom_center.x < 0) g_zoom_center.x = 0;
                            if (g_zoom_center.y < 0) g_zoom_center.y = 0;
                            if (g_zoom_center.x >= cW) g_zoom_center.x = cW - 1;
                            if (g_zoom_center.y >= cH) g_zoom_center.y = cH - 1;
                        }
                        updated = true;
                    }
                }

                if (updated) {
                    updateZoomedDisplay("distorted vs rectified with depth/confidence");
                }
            }

            if (!paused) break;
        }

        if (quit) break;
    }

    if (!had_messages) {
        std::cerr << "No RGB messages found in bag '" << rgb_bag_path
                  << "' for topics:\n  image: " << rgb_image_topic;
        if (have_camera_info_topic) std::cerr << "\n  camera_info: " << rgb_info_topic;
        std::cerr << "\n";
    } else {
        if (have_camera_info_topic && num_infos == 0) {
            std::cerr << "No valid sensor_msgs/CameraInfo messages received on topic "
                      << rgb_info_topic << "\n";
        }
        if (num_images == 0) {
            std::cerr << "No valid RGB messages received on topic " << rgb_image_topic << "\n";
        }
    }

    rgb_bag.close();
    if (use_depth) depth_bag.close();
    if (use_confidence && !confidence_uses_depth_bag) confidence_bag.close();
    return 0;
}
