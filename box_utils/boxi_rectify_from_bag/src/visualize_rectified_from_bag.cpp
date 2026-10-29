#include <rectify_from_bag/rectify_from_bag.hpp>
#include <rectify_from_bag/turbojpeg_utils.hpp>

#include <sensor_msgs/image_encodings.h>

#include <limits>
#include <string>
#include <vector>

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

static bool decodeRgbBgr(const rosbag::MessageInstance& msg,
                         const std::string& topic,
                         cv::Mat& rgb_bgr,
                         ros::Time& stamp_out)
{
    if (auto img_raw = msg.instantiate<sensor_msgs::Image>()) {
        stamp_out = img_raw->header.stamp;

        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(*img_raw, sensor_msgs::image_encodings::BGR8);
        } catch (const cv_bridge::Exception& e) {
            std::cerr << "cv_bridge exception (RGB Image) on "
                      << topic << ": " << e.what() << "\n";
            return false;
        }

        rgb_bgr = cv_ptr->image;
        return !rgb_bgr.empty();
    }

    if (auto img_cmp = msg.instantiate<sensor_msgs::CompressedImage>()) {
        stamp_out = img_cmp->header.stamp;

        if (!rectify_from_bag::turbojpeg_utils::decodeCompressedBGR(*img_cmp, rgb_bgr)) {
            std::cerr << "JPEG decode failed for RGB on topic "
                      << topic << "; skipping\n";
            return false;
        }

        return !rgb_bgr.empty();
    }

    std::cerr << "Message on topic " << topic
              << " is not sensor_msgs/Image or sensor_msgs/CompressedImage; skipping\n";
    return false;
}

static void putPanelTitle(cv::Mat& canvas,
                          int panel_index,
                          int panel_width,
                          const std::string& title)
{
    cv::putText(canvas,
                title,
                cv::Point(panel_index * panel_width + 30, 40),
                cv::FONT_HERSHEY_SIMPLEX,
                1.0,
                CV_RGB(255, 255, 255),
                2,
                cv::LINE_AA);
}

static void putPanelStatus(cv::Mat& canvas,
                           int panel_index,
                           int panel_width,
                           int panel_height,
                           const std::string& status)
{
    cv::putText(canvas,
                status,
                cv::Point(panel_index * panel_width + 30, panel_height - 30),
                cv::FONT_HERSHEY_SIMPLEX,
                0.8,
                CV_RGB(255, 160, 160),
                2,
                cv::LINE_AA);
}

static double wallSecondsNow()
{
    return ros::WallTime::now().toSec();
}

static void maybePrintRectifiedCameraInfo(const sensor_msgs::CameraInfo& info,
                                          bool have_info,
                                          double interval_sec,
                                          double& last_print_wall_sec)
{
    if (!have_info || interval_sec <= 0.0) return;

    const double now_sec = wallSecondsNow();
    if (std::isfinite(last_print_wall_sec) &&
        (now_sec - last_print_wall_sec) < interval_sec) {
        return;
    }

    last_print_wall_sec = now_sec;
    printCameraInfoMsg(info, "Rectified RGB CameraInfo");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualize_rectified_from_bag");
    ros::NodeHandle pnh("~");

    std::string rgb_bag_path;
    std::string rgb_image_topic;
    if (!requireParamNonEmpty(pnh, "rgb_bag", rgb_bag_path)) return 1;
    if (!requireParamNonEmpty(pnh, "rgb_image_topic", rgb_image_topic)) return 1;

    std::string rgb_info_bag_path;
    std::string rgb_info_topic;
    double rgb_info_print_interval_sec = 2.0;
    pnh.param<std::string>("rgb_info_bag", rgb_info_bag_path, std::string(""));
    pnh.param<std::string>("rgb_info_topic",
                           rgb_info_topic,
                           std::string("/boxi/zed2i/right/camera_info"));
    pnh.param("rgb_info_print_interval_sec",
              rgb_info_print_interval_sec,
              2.0);
    const bool use_rgb_info = !rgb_info_topic.empty();
    if (use_rgb_info && rgb_info_bag_path.empty()) {
        rgb_info_bag_path = rgb_bag_path;
    }

    bool use_depth = false;
    pnh.param("use_depth", use_depth, false);

    std::string depth_bag_path;
    std::string depth_image_topic;
    if (use_depth) {
        if (!requireParamNonEmpty(pnh, "depth_bag", depth_bag_path)) return 1;
        if (!requireParamNonEmpty(pnh, "depth_image_topic", depth_image_topic)) return 1;
    }

    bool use_confidence = false;
    pnh.param("use_confidence", use_confidence, false);

    std::string confidence_bag_path;
    std::string confidence_image_topic;
    pnh.param<std::string>("confidence_bag", confidence_bag_path, std::string(""));
    pnh.param<std::string>("confidence_image_topic",
                           confidence_image_topic,
                           std::string("/boxi/zed2i/confidence/image_rect/compressed"));

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

    rosbag::Bag rgb_bag;
    try {
        rgb_bag.open(rgb_bag_path, rosbag::bagmode::Read);
    } catch (const rosbag::BagException& e) {
        std::cerr << "Failed to open RGB bag: " << e.what() << "\n";
        return 1;
    }

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
        depth_it = depth_view.begin();
        depth_end = depth_view.end();
    }

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

        conf_it = confidence_view.begin();
        conf_end = confidence_view.end();
    }

    bool rgb_info_uses_rgb_bag = false;
    rosbag::Bag rgb_info_bag;
    rosbag::View rgb_info_view;
    rosbag::View::const_iterator rgb_info_it;
    rosbag::View::const_iterator rgb_info_end;
    if (use_rgb_info) {
        rgb_info_uses_rgb_bag = (rgb_info_bag_path == rgb_bag_path);
        if (rgb_info_uses_rgb_bag) {
            rgb_info_view.addQuery(rgb_bag, rosbag::TopicQuery({rgb_info_topic}));
        } else {
            try {
                rgb_info_bag.open(rgb_info_bag_path, rosbag::bagmode::Read);
            } catch (const rosbag::BagException& e) {
                std::cerr << "Failed to open RGB CameraInfo bag: " << e.what() << "\n";
                return 1;
            }

            rgb_info_view.addQuery(rgb_info_bag, rosbag::TopicQuery({rgb_info_topic}));
        }

        rgb_info_it = rgb_info_view.begin();
        rgb_info_end = rgb_info_view.end();
    }

    rosbag::View rgb_view(rgb_bag, rosbag::TopicQuery({rgb_image_topic}));

    const std::string window_name = "rectified RGB visualization";
    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    cv::setMouseCallback(window_name, onMouse, nullptr);

    std::cout << ANSI_ORANGE
              << "Read-only visualization: bags are opened with rosbag::bagmode::Read and no files are written.\n"
              << "Controls: p=Pause/Resume, q/ESC=Quit, z=Toggle zoom (while paused),\n"
              << "          +/-=Zoom in/out, WASD=Pan (while paused & zoom enabled),\n"
              << "          Left-click=move zoom center (while paused & zoom enabled)\n"
              << ANSI_RESET;

    bool paused = false;
    bool quit = false;
    bool window_initialized = false;
    bool had_messages = false;
    bool have_rectified_rgb_cam_info = false;
    size_t num_rgb_infos = 0;
    size_t num_images = 0;
    double last_rgb_info_print_wall_sec = -std::numeric_limits<double>::infinity();
    sensor_msgs::CameraInfo rectified_rgb_cam_info;

    for (const rosbag::MessageInstance& m : rgb_view) {
        had_messages = true;
        if (!topicMatches(m.getTopic(), rgb_image_topic)) continue;

        cv::Mat rgb;
        ros::Time rgb_stamp;
        if (!decodeRgbBgr(m, m.getTopic(), rgb, rgb_stamp)) {
            continue;
        }
        if (rgb.empty()) {
            std::cerr << "Decoded RGB image is empty; skipping\n";
            continue;
        }

        ++num_images;

        if (use_rgb_info && rgb_info_view.size() > 0) {
            while (rgb_info_it != rgb_info_end) {
                auto info = rgb_info_it->instantiate<sensor_msgs::CameraInfo>();
                if (!info) {
                    ++rgb_info_it;
                    continue;
                }

                const ros::Time info_stamp = info->header.stamp;
                if (info_stamp <= rgb_stamp) {
                    rectified_rgb_cam_info = *info;
                    have_rectified_rgb_cam_info = true;
                    ++num_rgb_infos;
                    ++rgb_info_it;
                    continue;
                }
                break;
            }
        }

        cv::Mat depth_color;
        bool have_depth_for_frame = false;
        if (use_depth && depth_view.size() > 0) {
            while (depth_it != depth_end) {
                ros::Time depth_stamp;
                if (!getStampFromMsg(*depth_it, depth_stamp)) {
                    ++depth_it;
                    continue;
                }

                if (depth_stamp < rgb_stamp) {
                    ++depth_it;
                    continue;
                }
                if (depth_stamp == rgb_stamp) {
                    if (decodeAndColorizeDepth(*depth_it, rgb.size(), depth_color)) {
                        ensureSameSize(rgb, depth_color, cv::INTER_NEAREST);
                        have_depth_for_frame = !depth_color.empty();
                    }
                    ++depth_it;
                    break;
                }
                break;
            }
        }

        cv::Mat conf_color;
        bool have_conf_for_frame = false;
        if (use_confidence && confidence_view.size() > 0) {
            while (conf_it != conf_end) {
                ros::Time conf_stamp;
                if (!getStampFromMsg(*conf_it, conf_stamp)) {
                    ++conf_it;
                    continue;
                }

                if (conf_stamp < rgb_stamp) {
                    ++conf_it;
                    continue;
                }
                if (conf_stamp == rgb_stamp) {
                    if (decodeAndColorizeDepth(*conf_it, rgb.size(), conf_color)) {
                        ensureSameSize(rgb, conf_color, cv::INTER_NEAREST);
                        have_conf_for_frame = !conf_color.empty();
                    }
                    ++conf_it;
                    break;
                }
                break;
            }
        }

        std::vector<cv::Mat> panels;
        std::vector<std::string> titles;
        std::vector<std::string> statuses;

        panels.push_back(rgb);
        titles.emplace_back("Rectified RGB");
        statuses.emplace_back("");

        if (use_depth) {
            cv::Mat rgb_with_depth = rgb.clone();
            if (have_depth_for_frame && !depth_color.empty()) {
                cv::addWeighted(depth_color, 0.6, rgb, 0.4, 0.0, rgb_with_depth);
            }

            panels.push_back(rgb_with_depth);
            titles.emplace_back("RGB + depth");
            statuses.emplace_back(have_depth_for_frame ? "" : "no exact stamp match");
        }

        if (use_confidence) {
            cv::Mat rgb_with_confidence = rgb.clone();
            if (have_conf_for_frame && !conf_color.empty()) {
                cv::addWeighted(conf_color, 0.6, rgb, 0.4, 0.0, rgb_with_confidence);
            }

            panels.push_back(rgb_with_confidence);
            titles.emplace_back("RGB + confidence");
            statuses.emplace_back(have_conf_for_frame ? "" : "no exact stamp match");
        }

        const int panel_width = rgb.cols;
        const int panel_height = rgb.rows;
        const int num_panels = static_cast<int>(panels.size());
        cv::Mat canvas(panel_height,
                       panel_width * num_panels,
                       rgb.type(),
                       cv::Scalar(0, 0, 0));

        for (int i = 0; i < num_panels; ++i) {
            panels[i].copyTo(canvas(cv::Rect(i * panel_width, 0, panel_width, panel_height)));
            putPanelTitle(canvas, i, panel_width, titles[i]);
            if (!statuses[i].empty()) {
                putPanelStatus(canvas, i, panel_width, panel_height, statuses[i]);
            }
        }

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
            cv::resizeWindow(window_name, canvas.cols, canvas.rows);
            window_initialized = true;
        }

        updateZoomedDisplay(window_name);
        maybePrintRectifiedCameraInfo(rectified_rgb_cam_info,
                                      have_rectified_rgb_cam_info,
                                      rgb_info_print_interval_sec,
                                      last_rgb_info_print_wall_sec);

        g_paused_global = paused;
        while (true) {
            const int key = cv::waitKey(30);

            maybePrintRectifiedCameraInfo(rectified_rgb_cam_info,
                                          have_rectified_rgb_cam_info,
                                          rgb_info_print_interval_sec,
                                          last_rgb_info_print_wall_sec);

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
                    } else if (!g_has_canvas || g_current_canvas.empty()) {
                        g_zoom_enabled = false;
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
                            const int width = g_current_canvas.cols;
                            const int height = g_current_canvas.rows;
                            if (g_zoom_center.x < 0) g_zoom_center.x = 0;
                            if (g_zoom_center.y < 0) g_zoom_center.y = 0;
                            if (g_zoom_center.x >= width) g_zoom_center.x = width - 1;
                            if (g_zoom_center.y >= height) g_zoom_center.y = height - 1;
                        }
                        updated = true;
                    }
                }

                if (updated) {
                    updateZoomedDisplay(window_name);
                }
            }

            if (!paused) break;
        }

        if (quit) break;
    }

    if (!had_messages) {
        std::cerr << "No RGB messages found in bag '" << rgb_bag_path
                  << "' for topic " << rgb_image_topic << "\n";
    } else if (num_images == 0) {
        std::cerr << "No valid RGB messages received on topic "
                  << rgb_image_topic << "\n";
    } else if (use_rgb_info && num_rgb_infos == 0) {
        std::cerr << "No valid rectified RGB CameraInfo messages received on topic "
                  << rgb_info_topic << "\n";
    }

    rgb_bag.close();
    if (use_depth) depth_bag.close();
    if (use_confidence && !confidence_uses_depth_bag) confidence_bag.close();
    if (use_rgb_info && !rgb_info_uses_rgb_bag) rgb_info_bag.close();

    return 0;
}
