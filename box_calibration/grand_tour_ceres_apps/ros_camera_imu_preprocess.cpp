// Offline preprocessor: reads raw image bags, runs april-grid detection,
// and writes grand_tour_camera_detection_msgs/CameraDetections to an output bag.
// Output topic: {image_topic}_corner_detections   (mirrors detector_node convention)
//
// Usage:
//   ros_camera_imu_preprocess --bags bag1.bag bag2.bag --output detections.bag

#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <grand_tour_camera_detection_msgs/CameraDetections.h>

#include <grand_tour_camera_detectors/aprilgridasl.h>
#include <gtboxcalibration/argparsers.h>

namespace fs = std::filesystem;

static constexpr int    kGridSizeX   = 6;
static constexpr int    kGridSizeY   = 6;
static constexpr double kTagSize     = 0.083;
static constexpr double kTagSpacing  = 0.3;
static constexpr char   kDetSuffix[] = "_corner_detections";

int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_camera_imu_preprocess");

    argparse::ArgumentParser program("ros_camera_imu_preprocess");
    program.add_argument("--bags")
        .help("Input rosbag paths containing sensor_msgs/Image topics.")
        .nargs(argparse::nargs_pattern::any)
        .default_value(std::vector<std::string>{})
        .action([](const std::string& v) { return v; });
    program.add_argument("-o", "--output")
        .required()
        .help("Output rosbag path for CameraDetections messages.");

    try {
        program.parse_args(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n" << program;
        return 1;
    }

    const auto bag_paths = program.get<std::vector<std::string>>("--bags");
    const auto output_path = program.get<std::string>("--output");

    if (bag_paths.empty()) {
        std::cerr << "No input bags provided.\n";
        return 1;
    }

    auto options = cameras::GridCalibrationTargetAprilgrid::AprilgridOptions();
    cameras::GridCalibrationTargetAprilgrid detector(kGridSizeX, kGridSizeY,
                                                     kTagSize, kTagSpacing, options);

    rosbag::Bag out_bag;
    out_bag.open(output_path, rosbag::bagmode::Write);

    size_t n_images = 0, n_detections = 0;

    for (const auto& path : bag_paths) {
        if (!fs::exists(path)) {
            std::cerr << "Bag not found: " << path << "\n";
            continue;
        }
        rosbag::Bag bag;
        bag.open(path, rosbag::bagmode::Read);
        std::cout << "Processing: " << path << std::endl;

        rosbag::View view(bag);
        const size_t total_msgs = view.size();
        size_t msg_idx = 0;
        std::unordered_map<std::string, bool> do_write_image;

        for (const auto& m : view) {
            ++msg_idx;

            auto img_msg = m.instantiate<sensor_msgs::Image>();
            if (!img_msg) continue;

            const std::string& image_topic = m.getTopic();
            if (do_write_image.find(image_topic) == do_write_image.end()) {
                do_write_image[image_topic] = true;
            }

            ++n_images;
            if (n_images % 50 == 0) {
                const int pct = static_cast<int>(100.0 * msg_idx / total_msgs);
                std::cout << "\r  [" << pct << "%] " << n_images
                          << " images / " << n_detections << " detections"
                          << std::flush;
                for (auto& [topic, seen]: do_write_image) {
                    do_write_image[topic] = true;
                }
            }

            if (do_write_image.at(image_topic)) {
                out_bag.write(image_topic, img_msg->header.stamp, *img_msg);
                do_write_image[image_topic] = false;
            }

            cv::Mat image;
            try {
                image = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8)->image;
            } catch (const cv_bridge::Exception& e) {
                ROS_WARN("cv_bridge: %s", e.what());
                continue;
            }

            Eigen::MatrixXd corners;
            std::vector<bool> valid;
            detector.computeObservation(image, corners, valid);

            grand_tour_camera_detection_msgs::CameraDetections det_msg;
            det_msg.header = img_msg->header;

            for (size_t i = 0; i < valid.size(); ++i) {
                if (!valid[i]) continue;

                geometry_msgs::Point c2d;
                c2d.x = corners(i, 0);
                c2d.y = corners(i, 1);
                c2d.z = 0.0;
                det_msg.corners2d.push_back(c2d);

                const Eigen::Vector3d pt = detector.point(i);
                geometry_msgs::Point mpt;
                mpt.x = pt.x(); mpt.y = pt.y(); mpt.z = pt.z();
                det_msg.modelpoint3d.push_back(mpt);

                det_msg.cornerids.push_back(static_cast<int32_t>(i));
            }

            if (det_msg.cornerids.empty()) continue;

            ++n_detections;
            out_bag.write(image_topic + kDetSuffix, img_msg->header.stamp, det_msg);
        }
        bag.close();
        std::cout << "\n";
    }

    out_bag.close();
    std::cout << "Done. Images processed: " << n_images
              << "  Frames with detections: " << n_detections
              << "  Written to: " << output_path << std::endl;
    return 0;
}