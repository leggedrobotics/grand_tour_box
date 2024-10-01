#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Time.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>
#include <std_srvs/SetBool.h>
#include <deque>
#include <fstream>
#include <thread>
#include <mutex>
#include <vector>
#include <cmath>
#include <map>
#include <signal.h>
#include <algorithm>
#include <memory>
#include <ap20_driver_ros/TimestampDebug.h>
#include <ap20_driver_ros/PositionDebug.h>
#include <ap20_driver_ros/ImuDebug.h>
#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <string>
#include <array>

extern const std::string timestamps_fn;
const std::string timestamper_module = "time_stamper";
const std::string timestamps_fn = std::string("/sys/kernel/") + timestamper_module + "/ts_buffer";

class AP20Node {
public:
    AP20Node(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
        : nh_(nh), private_nh_(private_nh), state_(RESTARTING_AP20), restart_streaming_attempts_(0), steps_without_timestamps_(0), steps_without_imu_messages_(0) {
        timestamp_counter_ = 0;
        position_counter_ = 0;
        imu_counter_ = 0;
        last_imu_counter_ = 0;
        last_timestamp_counter_ = 0;

        timestamp_reader_ = std::make_unique<ContinuousTimestampReader>("/sys/kernel/time_stamper/ts_buffer");
        initializeCallbacks();
    }

    void run();

private:
    enum State {
        RESTARTING_AP20,
        RESTARTING_STREAMING,
        WAITING_FOR_FIRST_MESSAGE,
        RUNNING
    };

    State state_;
    std::atomic<int> restart_streaming_attempts_;
    std::atomic<int> steps_without_timestamps_;
    std::atomic<int> steps_without_imu_messages_;
    std::atomic<int> last_imu_counter_;
    std::atomic<int> last_timestamp_counter_;

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void positionCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
    void timerCallback(const ros::TimerEvent& event);
    void monitor(const ros::TimerEvent& event);
    void changeImuMode(bool enable_streaming);
    void runCommandAsyncNoReturn(const std::string& command);

    class ContinuousTimestampReader {
    public:
        ContinuousTimestampReader(const std::string& timestamps_fn);
        std::vector<ros::Time> readTimestamps();
    private:
        void openFile();
        std::string timestamps_fn_;
        std::ifstream timestamps_file_;
    };

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber imu_sub_;
    ros::Subscriber position_sub_;
    ros::Publisher timestamp_debug_pub_;
    ros::Publisher imu_debug_pub_;
    ros::Publisher position_debug_pub_;
    ros::Timer timestamp_sub_;

    ros::Timer state_monitor_;
    
    std::unique_ptr<ContinuousTimestampReader> timestamp_reader_;

    std::atomic<int> timestamp_counter_;
    std::atomic<int> position_counter_;
    std::atomic<int> imu_counter_;

    void initializeCallbacks() {
        timestamp_debug_pub_ = nh_.advertise<ap20_driver_ros::TimestampDebug>("ap20/timestamp_debug", 1000);
        position_debug_pub_ = nh_.advertise<ap20_driver_ros::PositionDebug>("ap20/position_debug", 1000);
        imu_debug_pub_ = nh_.advertise<ap20_driver_ros::ImuDebug>("ap20/imu_debug", 1000);

        imu_sub_ = nh_.subscribe("/ap20/imu", 1000, &AP20Node::imuCallback, this);
        position_sub_ = nh_.subscribe("/ap20/tps", 1000, &AP20Node::positionCallback, this);
        timestamp_sub_ = nh_.createTimer(ros::Duration(0.001), &AP20Node::timerCallback, this);

        state_monitor_ = nh_.createTimer(ros::Duration(0.001), &AP20Node::monitor, this);
    }
};

// Implementation of ContinuousTimestampReader methods
AP20Node::ContinuousTimestampReader::ContinuousTimestampReader(const std::string& timestamps_fn)
    : timestamps_fn_(timestamps_fn) {
    openFile();
}

void AP20Node::ContinuousTimestampReader::openFile() {
    if (!timestamps_file_.is_open()) {
        timestamps_file_.open(timestamps_fn_);
        if (!timestamps_file_.is_open()) {
            ROS_ERROR_STREAM("Failed to open timestamps file: " << timestamps_fn_);
        }
        ROS_INFO_STREAM("Opened timestamps file: " << timestamps_fn_);
    }
}

std::vector<ros::Time> AP20Node::ContinuousTimestampReader::readTimestamps() {
    std::vector<ros::Time> ret;
    if (!timestamps_file_.is_open()) {
        openFile();
        if (!timestamps_file_.is_open()) {
            return ret;
        }
    }

    timestamps_file_.clear();
    timestamps_file_.seekg(0);
    std::string line;
    while (std::getline(timestamps_file_, line)) {
        try {
            size_t dot_pos = line.find('.');
            if (dot_pos != std::string::npos) {
                int32_t secs = std::stoi(line.substr(0, dot_pos));
                int32_t nsecs = std::stoi(line.substr(dot_pos + 1));
                ret.emplace_back(secs, nsecs);
            }
        } catch (const std::exception& e) {
            ROS_WARN("Unexpected format in timestamps file");
        }
    }

    return ret;
}

void AP20Node::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    ROS_INFO_STREAM_THROTTLE(5.0, "Publish IMU Message (throttled 5s): " << imu_counter_);
    ap20_driver_ros::ImuDebug debug_imu;
    debug_imu.header.stamp = ros::Time::now();
    debug_imu.imu = *msg;
    int tmp = imu_counter_.load();
    debug_imu.header.seq = tmp;
    imu_debug_pub_.publish(debug_imu);

    imu_counter_++;
}

void AP20Node::positionCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    ROS_INFO_STREAM_THROTTLE(5.0, "Publish Position Message (throttled 5s): " << position_counter_);
    ap20_driver_ros::PositionDebug debug_position;
    debug_position.header.stamp = ros::Time::now();
    debug_position.position = *msg;
    int tmp = position_counter_.load()
    debug_position.header.seq = tmp;
    position_debug_pub_.publish(debug_position);

    position_counter_++;
}

void AP20Node::timerCallback(const ros::TimerEvent&) {
    auto stamps = timestamp_reader_->readTimestamps();
    
    for (const auto& ts : stamps) {
        ROS_INFO_STREAM_THROTTLE(5.0, "New Timestamp Message (throttled 5s): " << timestamp_counter_);

        
        ap20_driver_ros::TimestampDebug debug_timestamp;
        debug_timestamp.header.stamp = ros::Time::now();
        int tmp = timestamp_counter_.load()
        debug_timestamp.header.seq = tmp;
        debug_timestamp.timestamp.data = ts;
        timestamp_debug_pub_.publish(debug_timestamp);

        timestamp_counter_++;
    }
}

void AP20Node::runCommandAsyncNoReturn(const std::string& command) {
    std::thread([command]() {
        auto start_time = std::chrono::high_resolution_clock::now();
        int result = system(command.c_str());
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_time = end_time - start_time;

        if (result == 0) {
            ROS_INFO("Command executed successfully");
        } else {
            ROS_ERROR("Command failed, execution failed with code %d", result);
        }

        // Log the time taken
        ROS_INFO_STREAM("Async command {" << command << "} took " << elapsed_time.count() << " seconds");

    }).detach();  // Detach the thread so that it doesn't block the main thread
}

void AP20Node::changeImuMode(bool enable_streaming) {
    std::string imu_command;
    if (enable_streaming) {
        imu_command = "rosservice call /ap20/enable_streaming \"{data: true}\"";
    } else {
        imu_command = "rosservice call /ap20/enable_streaming \"{data: false}\"";
    }

    // Run the command asynchronously (fire-and-forget, no return value)
    runCommandAsyncNoReturn(imu_command);
}

void AP20Node::monitor(const ros::TimerEvent&) {
    // Process the current state machine state
    switch (state_) {
        case RESTARTING_AP20: {
            ROS_INFO("STATE: RESTARTING_AP20");
            std::string start = "/home/rsl/git/grand_tour_box/box_drivers/ap20_driver/ap20_driver_ros/script/ap20_start_rosrover.sh";
            runCommandAsyncNoReturn(start);
            state_ = RESTARTING_STREAMING;
            break;
        }

        case RESTARTING_STREAMING: {
            ROS_INFO("STATE: RESTARTING_STREAMING");
            if (restart_streaming_attempts_ <= 10) {
                ROS_INFO_STREAM("Restarting Streaming - attempt: " << restart_streaming_attempts_);
                changeImuMode(false);
                imu_counter_ = 0;
                position_counter_ = 0;
                int timestamp_counter_before_wait = timestamp_counter_;
                int imu_counter_before_wait = imu_counter_;
                timestamp_counter_ = 0;

                ros::Duration(0.5).sleep();

                if (timestamp_counter_ == timestamp_counter_before_wait && imu_counter_ == imu_counter_before_wait) {
                    changeImuMode(true);
                    ROS_INFO("No messages received in 0.5 seconds - stop sucessful, restarting streaming.");
                    state_ = WAITING_FOR_FIRST_MESSAGE;
                    restart_streaming_attempts_ = 0;
                } else {
                    restart_streaming_attempts_++;
                }
            } else {
                ROS_INFO("Max restart attempts reached, going back to RESTARTING_AP20.");
                state_ = RESTARTING_AP20;
            }
            break;
        }

        case WAITING_FOR_FIRST_MESSAGE: {
            ROS_INFO_THROTTLE(0.5, "STATE: WAITING_FOR_FIRST_MESSAGE");
            if (imu_counter_ > 0) {
                state_ = RUNNING;
                steps_without_imu_messages_ = 0;
            } else {
                steps_without_imu_messages_++;
                if (steps_without_imu_messages_ % 500 == 0) {
                    changeImuMode(true);
                }
                if (steps_without_imu_messages_ >= 20000) {  // 20 seconds timeout
                    ROS_INFO("No IMU message received for 20 seconds. Restarting AP20.");
                    state_ = RESTARTING_AP20;
                }
            }
            break;
        }

        case RUNNING: {
            ROS_INFO_THROTTLE(5.0, "STATE: RUNNING");
            if (imu_counter_ > last_imu_counter_ || timestamp_counter_ > last_timestamp_counter_) {
                last_imu_counter_ = imu_counter_.load();
                last_timestamp_counter_ = timestamp_counter_.load();
                steps_without_imu_messages_ = 0;
                // Warn if we missed any messages, this monitor thread runs at 1kHz and imu messages are published at 200Hz
                if (imu_counter_ > last_imu_counter_ + 1 || timestamp_counter_ > last_timestamp_counter_ + 1) {
                    ROS_WARN_STREAM("Monitor thread missed at least one IMU or timestamp message. imu_counter: " << 
                                imu_counter_ << ", last_imu_counter: " << last_imu_counter_ << ", timestamp_counter: " << 
                                timestamp_counter_ << ", last_timestamp_counter: " << last_timestamp_counter_);
                }
            } else {
                steps_without_imu_messages_ += 1;
                if (steps_without_imu_messages_ >= 2000) {
                    ROS_WARN("No new IMU or timestamp messages for 2 seconds, restarting streaming.");
                    state_ = RESTARTING_STREAMING;
                }
            }
            break;
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ap20");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    AP20Node node(nh, private_nh);
    // Spin ROS so callbacks can be called
    ros::spin();

    return 0;
}
