#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <deque>
#include <chrono>
#include <sl/Camera.hpp>
#include <zed2i_recording_driver_ros/start_recording_svo.h>

class ZedRecordingNode {
private:
    ros::NodeHandle nh_;
    ros::Publisher hz_pub_;
    ros::ServiceServer service_;
    std::vector<std::thread> threads_;
    std::deque<sl::Timestamp> timestamps_;
    std::mutex mutex_;
    std::atomic<bool> running_{true};
    std::atomic<bool> recording_{false};
    sl::Camera zed_;
    std::string video_filename_;

public:
    ZedRecordingNode() : nh_("~") {
        hz_pub_ = nh_.advertise<std_msgs::Float32>("recording_hz", 10);
        service_ = nh_.advertiseService("start_recording_svo", &ZedRecordingNode::recordingService, this);
        ROS_INFO("Initializing ZedRecordingNode...");
        openCamera();
    }

    ~ZedRecordingNode() {
        running_ = false;
        for (auto& thread : threads_) {
            if (thread.joinable()) {
                thread.join();
            }
        }
        zed_.close();
    }

    void start() {
        threads_.emplace_back(&ZedRecordingNode::publishRecordingHz, this);
        threads_.emplace_back(&ZedRecordingNode::recordSvo, this);
        ros::spin();
    }

private:
    sl::RESOLUTION getResolutionFromString(const std::string& resolution) {
        if (resolution == "VGA") {
            return sl::RESOLUTION::VGA;
        } else if (resolution == "HD720") {
            return sl::RESOLUTION::HD720;
        } else if (resolution == "HD1080") {
            return sl::RESOLUTION::HD1080;
        } else if (resolution == "HD2K") {
            return sl::RESOLUTION::HD2K;
        } else {
            ROS_WARN("Unknown resolution setting '%s'. Defaulting to HD1080.", resolution.c_str());
            return sl::RESOLUTION::HD1080;
        }
    }

    void openCamera() {
        std::string camera_resolution;
        nh_.param<std::string>("camera_resolution", camera_resolution, "HD1080");
        int camera_fps;
        nh_.param<int>("camera_fps", camera_fps, 30);

        sl::InitParameters init_params;
        init_params.camera_resolution = getResolutionFromString(camera_resolution);
        init_params.depth_mode = sl::DEPTH_MODE::NONE;
        init_params.camera_fps = camera_fps;
        init_params.input.setFromCameraID(0);

        sl::ERROR_CODE err = zed_.open(init_params);
        if (err != sl::ERROR_CODE::SUCCESS) {
            ROS_FATAL_STREAM("ZED camera cannot be opened, error: " << sl::toString(err));
            ros::shutdown();
            return;
        }
        ROS_INFO_STREAM("ZED camera opened successfully. Resolution: " << init_params.camera_resolution << ", FPS: " << init_params.camera_fps);
    }

    bool recordingService(zed2i_recording_driver_ros::start_recording_svo::Request &req,
                               zed2i_recording_driver_ros::start_recording_svo::Response &res) {
        recording_ = req.start_recording;
        video_filename_ = req.video_filename;
        ROS_INFO_STREAM("Recording service called. Recording: " << (recording_ ? "STARTED" : "STOPPED") << ", Filename: " << video_filename_);
        res.success = true;
        return true;
    }

    void publishRecordingHz() {
        while (ros::ok()) {
            if (!recording_) continue;
            std_msgs::Float32 msg;
            uint64_t time_span_nanos = 0;
            int window_size = 10;

            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (timestamps_.size() == window_size) {
                    time_span_nanos = timestamps_.back().getNanoseconds() - timestamps_.front().getNanoseconds();
                }
            }

            if (time_span_nanos == 0) {
                continue;
            }

            double avg_hz = 1 / ((time_span_nanos / 1e9) / (window_size -1));  // Calculate Hz
            msg.data = static_cast<float>(avg_hz);
            hz_pub_.publish(msg);
            ros::Duration(1.0).sleep();  // Publish at 1 Hz
            ROS_INFO_STREAM_THROTTLE(10, "Publishing recording frequency: " << msg.data << " Hz");
        }
    }

    void recordSvo() {
        while (!recording_){
            ROS_INFO_STREAM_THROTTLE(5, "Waiting for recording to start...");
            ros::Duration(0.1).sleep();
        }
        sl::RecordingParameters recording_params;
        recording_params.compression_mode = sl::SVO_COMPRESSION_MODE::H264;
        recording_params.video_filename = sl::String(video_filename_.c_str());

        auto err = zed_.enableRecording(recording_params);
        if (err != sl::ERROR_CODE::SUCCESS) {
            ROS_FATAL_STREAM("Failed to start recording, error: " << sl::toString(err) << ". shutting down the node.");
            ros::shutdown();
            return;
        }
        ROS_INFO_STREAM("Recording started.");

        while (running_ && recording_) {
            if (zed_.grab() == sl::ERROR_CODE::SUCCESS) {
                auto grab_ts = zed_.getTimestamp(sl::TIME_REFERENCE::IMAGE);
                std::lock_guard<std::mutex> lock(mutex_);
                timestamps_.push_back(grab_ts);
                if (timestamps_.size() > 10) {
                    timestamps_.pop_front();
                }
            }
        }
        zed_.disableRecording();
        ROS_INFO_STREAM("Recording ended.");
        if (running_) {
            recordSvo();  // Restart waiting for recording if still running
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "zed2i_recording_driver");
    ZedRecordingNode node;
    node.start();
    return 0;
}
