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
        : nh_(nh), private_nh_(private_nh) {
        timestamp_counter_ = 0;
        position_counter_ = 0;
        imu_counter_ = 0;
        publish_ = false;

        initializePublishersAndSubscribers();
        timestamp_reader_ = std::make_unique<ContinuousTimestampReader>("/sys/kernel/time_stamper/ts_buffer");
    }

    void run();

private:

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void positionCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
    bool changeImuMode(bool enable_streaming);

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
    // DEBUG
    ros::Publisher timestamp_debug_pub_;
    ros::Publisher imu_debug_pub_;
    ros::Publisher position_debug_pub_;
    
    std::unique_ptr<ContinuousTimestampReader> timestamp_reader_;
    int timestamp_counter_;
    int position_counter_;
    int imu_counter_;

    bool publish_;

    

    void initializePublishersAndSubscribers() {
        // DEBUG
        timestamp_debug_pub_ = nh_.advertise<ap20_driver_ros::TimestampDebug>("ap20/timestamp_debug", 10);
        position_debug_pub_ = nh_.advertise<ap20_driver_ros::PositionDebug>("ap20/position_debug", 10);
        imu_debug_pub_ = nh_.advertise<ap20_driver_ros::ImuDebug>("ap20/imu_debug", 10);

        // Subscribers
        imu_sub_ = nh_.subscribe("/ap20/imu", 10, &AP20Node::imuCallback, this);
        position_sub_ = nh_.subscribe("/ap20/tps", 10, &AP20Node::positionCallback, this);

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
    
    
    if (publish_){
        ROS_INFO_STREAM("Publish IMU Message: " << imu_counter_);
        ap20_driver_ros::ImuDebug debug_imu;
        debug_imu.header.stamp = ros::Time::now();
        debug_imu.imu = *msg;
        debug_imu.header.seq = imu_counter_;
        imu_debug_pub_.publish(debug_imu);
    }else{
        ROS_INFO_STREAM("Received IMU Message: " << imu_counter_);
    }
    
    imu_counter_++;
}

void AP20Node::positionCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    
    if (publish_) {
        ROS_INFO_STREAM("Publish Position Message: " << position_counter_);
        ap20_driver_ros::PositionDebug debug_position;
        debug_position.header.stamp = ros::Time::now();
        debug_position.position = *msg;
        debug_position.header.seq = position_counter_;
        position_debug_pub_.publish(debug_position);
    }else{
        ROS_INFO_STREAM("Received Position Message: " << imu_counter_);
    }

    position_counter_++;
}

std::string runCommand(const std::string& command) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(command.c_str(), "r"), pclose);
    
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    
    return result;
}

// Function to run the command asynchronously
void runCommandAsync(const std::string& command) {
    std::thread([command]() {
        system(command.c_str());
    }).detach();  // Detach the thread to make it non-blocking
}

bool AP20Node::changeImuMode(bool enable_streaming) {
    std::string start_imu_command;
    if (enable_streaming) {
      start_imu_command = "rosservice call /ap20/enable_streaming \"{data: true}\"";
      runCommand(start_imu_command);
      return true;
    } else {
      start_imu_command = "rosservice call /ap20/enable_streaming \"{data: false}\"";
    }

    int result = system(start_imu_command.c_str());
    if (result == 0) {
      ROS_INFO("IMU mode changed successfully. Streaming is now %s.", enable_streaming ? "enabled" : "disabled");
      return true;
    } else {
      std::cerr << "Startung IMU failed, execution failed with code " << result << std::endl;
      return false;
    }
}

void AP20Node::run() {

    ros::Rate loop_rate(500);
    while (ros::ok()) {
        ROS_INFO("Restart AP20");

        std::string start = "/home/rsl/git/grand_tour_box/box_drivers/ap20_driver/ap20_driver_ros/script/ap20_start_rosrover.sh";
        runCommandAsync(start);
        
        int steps_without_timestamps = 0;
        int suc = false;
        int restart_streaming_attempts = 0;
        ROS_INFO("Restart Streaming");
        publish_ = false;
        while (true) {
            changeImuMode(false);
            auto cleared_timestamps = timestamp_reader_->readTimestamps();
            imu_counter_ = 0;
            position_counter_ = 0;
            timestamp_counter_ = 0;
            
            // this is needed to allow for collecting IMU messages
            for (int i = 0; i++; i < 500){
                ros::Duration(0.001).sleep();
                ros::spinOnce();
            }
            
            auto old_timestamps = timestamp_reader_->readTimestamps();
            ROS_INFO_STREAM("Timestamp queue size: " << old_timestamps.size());
            ROS_INFO_STREAM("IMU Message counter: " << imu_counter_);

            if (old_timestamps.empty() && imu_counter_ == 0) {
                suc = true;
                break;
            }
            restart_streaming_attempts++;
            ROS_INFO_STREAM("Timestamps still streaming - attempt: " << restart_streaming_attempts);
            if (restart_streaming_attempts == 40){
                break;
            }
        }
        if (!suc){
            ROS_INFO_STREAM("Reached max restart streaming attempts -> Restart AP20");
            continue;
        }

        
        changeImuMode(true);
        publish_ = true;
        imu_counter_ = 0;
        position_counter_ = 0;
        timestamp_counter_ = 0;
        
        ROS_INFO("Wait for first IMU message for total duration of 20s");
        steps_without_timestamps = 0;
        suc = false;
        while (true){
            steps_without_timestamps++;
            if (steps_without_timestamps > 20000) {
                break;
            }

            if (steps_without_timestamps % 500 == 0){
                changeImuMode(true);
            }
            ros::Duration(0.001).sleep();
            ros::spinOnce();
            if (imu_counter_ != 0){
                suc = true;
                break;
            }
        }
        if (!suc) {
            ROS_INFO("Restart the AP20 due to no IMU Message received for 20s");
            continue;
        } 

        ROS_INFO("Start normal operation");
        int k = 0;
        while (ros::ok()) {
            ros::spinOnce();

            auto stamps = timestamp_reader_->readTimestamps();
            
            if (stamps.empty()) {
                steps_without_timestamps++;
                if (steps_without_timestamps > 1000) {
                    ROS_ERROR("No timestamps for 1000 steps. Restarting streaming.");
                    break;
                }
            } else{
                steps_without_timestamps = 0;
                for (const auto& ts : stamps) {
                    ROS_INFO_STREAM("New Timestamp Message: " << timestamp_counter_);
                    
                    ap20_driver_ros::TimestampDebug debug_timestamp;
                    debug_timestamp.header.stamp = ros::Time::now();
                    debug_timestamp.header.seq = timestamp_counter_;
                    debug_timestamp.timestamp.data = ts;
                    timestamp_debug_pub_.publish(debug_timestamp);

                    timestamp_counter_++;
                }
            }
            loop_rate.sleep();
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ap20");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    AP20Node node(nh, private_nh);
    node.run();

    return 0;
}