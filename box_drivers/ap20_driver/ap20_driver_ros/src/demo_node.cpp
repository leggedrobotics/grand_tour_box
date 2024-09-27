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
#include <ap20_driver_ros/TimestampKappiDebug.h> 


extern const std::string timestamps_fn;

const std::string timestamper_module = "time_stamper";
const std::string timestamps_fn = std::string("/sys/kernel/") + timestamper_module + "/ts_buffer";

class AP20Node {
public:
    AP20Node(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
        : nh_(nh), private_nh_(private_nh) {
        initializeVariables();
        initializePublishersAndSubscribers();
        timestamp_reader_ = std::make_unique<ContinuousTimestampReader>("/sys/kernel/time_stamper/ts_buffer");
    }

    void run();

private:

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void positionCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
    void publishPositionMessages();
    void publishImuMessages();
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
    ros::Publisher ap20_imu_pub_;
    ros::Publisher ap20_position_pub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber position_sub_;
    // DEBUG
    ros::Publisher timestamp_debug_pub_;
    
    std::deque<sensor_msgs::ImuPtr> imu_message_queue_;
    std::deque<geometry_msgs::PointStampedPtr> position_message_queue_;
    

    std::unique_ptr<ContinuousTimestampReader> timestamp_reader_;

    std::deque<ros::Time> timestamp_queue_;
    // Mutexes
    std::mutex imu_message_queue_mutex_;
    std::mutex position_message_queue_mutex_;



    std::vector<double> lookup_seq_;
    std::vector<double> lookup_ts_;
    std::vector<double> line_delay_;

    bool publish_timestamps_;
    int imu_message_counter_;
    int timestamp_counter_;
    int internal_counter_imu_ap20_;
    bool internal_counter_imu_ap20_valid_;
    bool soft_shutdown_;
    int last_published_imu_seq_;
    bool backlog_cleared_;
    float last_imu_message_header_time;

    void initializeVariables() {
        publish_timestamps_ = false;
        imu_message_counter_ = 0;
        timestamp_counter_ = 0;
        internal_counter_imu_ap20_ = -1;
        internal_counter_imu_ap20_valid_ = false;
        soft_shutdown_ = false;
        last_published_imu_seq_ = -1;
        backlog_cleared_ = false;
        last_imu_message_header_time = -1;
    }

    void initializePublishersAndSubscribers() {
        ap20_imu_pub_ = nh_.advertise<sensor_msgs::Imu>("ap20/imu", 200);
        ap20_position_pub_ = nh_.advertise<geometry_msgs::PointStamped>("ap20/prism_position", 100);
        imu_sub_ = nh_.subscribe("/ap20/imu", 50, &AP20Node::imuCallback, this);
        position_sub_ = nh_.subscribe("/ap20/tps", 50, &AP20Node::positionCallback, this);
        // DEBUG
        timestamp_debug_pub_ = nh_.advertise<ap20_driver_ros::TimestampKappiDebug>("ap20/timestamp_debug", 10);
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

// Implementation of AP20Node methods
void AP20Node::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    ROS_INFO_STREAM("New IMU Message");

    std::lock_guard<std::mutex> lock(imu_message_queue_mutex_);
    imu_message_queue_.push_front(boost::make_shared<sensor_msgs::Imu>(*msg));


    if (msg->header.seq != internal_counter_imu_ap20_ + 1 && 
        internal_counter_imu_ap20_ != -1 && 
        !internal_counter_imu_ap20_valid_) {
        ROS_ERROR_STREAM("Missed message: " << internal_counter_imu_ap20_ << " -> " << msg->header.seq);
        soft_shutdown_ = true;
    }
    if (internal_counter_imu_ap20_valid_) {
        internal_counter_imu_ap20_ = msg->header.seq;
    }
    imu_message_counter_++;
}

void AP20Node::positionCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    ROS_INFO_STREAM("New Position Message");
    
    std::lock_guard<std::mutex> lock(position_message_queue_mutex_);
    position_message_queue_.push_front(boost::make_shared<geometry_msgs::PointStamped>(*msg));

}

void AP20Node::publishPositionMessages() {
    std::lock_guard<std::mutex> lock(position_message_queue_mutex_);
    if (!position_message_queue_.empty()) {
        auto position = position_message_queue_.back();
        position_message_queue_.pop_back();

        double ap20_original_ts = position->header.stamp.toSec();

        if (lookup_seq_.empty()) {
            ROS_ERROR("lookup seq is empty in pos calc.");
            return;
        }

        size_t j = 0;
        double delta = 0;
        for (size_t k = 0; k < lookup_seq_.size(); ++k) {
            delta = lookup_seq_[k] - ap20_original_ts;
            if (delta > 0) {
                j = k;
                break;
            }
        }

        if (delta > 0.01 + 1e-5) {
            ROS_ERROR("delta > 0.01 + 1e-5 in pos calc.");
            return;
        }

        if (j == 0) {
            ROS_ERROR("j == 0 in pos calc.");
            return;
        }

        if (j == lookup_ts_.size()) {
            position_message_queue_.push_back(position);
            return;
        }

        double rate = (ap20_original_ts - lookup_seq_[j - 1]) / (lookup_seq_[j] - lookup_seq_[j - 1]);
        double new_ts = lookup_ts_[j - 1] + ((lookup_ts_[j] - lookup_ts_[j - 1]) * rate);

        position->header.stamp = ros::Time(new_ts);
        ap20_position_pub_.publish(position);
        lookup_seq_.erase(lookup_seq_.begin(), lookup_seq_.begin() + j - 1);
        lookup_ts_.erase(lookup_ts_.begin(), lookup_ts_.begin() + j - 1);
    }
}

void AP20Node::publishImuMessages() {
    ros::Time time_msg;
    sensor_msgs::ImuPtr imu;
    {
        std::lock_guard<std::mutex> lock(imu_message_queue_mutex_);
        if (imu_message_queue_.empty() || timestamp_queue_.empty()) {
            return;
        }
        // ROS_INFO_STREAM("Timestamp queue size: " << timestamp_queue_.size());
        // ROS_INFO_STREAM("IMU message queue size: " << imu_message_queue_.size());

        time_msg = timestamp_queue_.back();
        timestamp_queue_.pop_back();
        imu = imu_message_queue_.back();
        imu_message_queue_.pop_back();  
    }

    if (last_imu_message_header_time != -1 && imu->header.stamp.toSec() < last_imu_message_header_time){
        ROS_ERROR("The totalstation fully disconnected from the AP20 - and successfully reconncted - as a result the AP20 starts counting again from 0 seconds - jumped back in time");
        line_delay_.clear();
        lookup_seq_.clear();
        lookup_ts_.clear();
        soft_shutdown_ = true;
        return;
    } 

    // DEBUG
    ap20_driver_ros::TimestampKappiDebug timestamp_debug_msg;
    timestamp_debug_msg.timestamp.data = time_msg;
    timestamp_debug_msg.ap20_timestamp.data = imu->header.stamp;
    auto now = ros::Time::now();
    timestamp_debug_msg.header.stamp = now;
    timestamp_debug_msg.node_arrival_delta = now.toSec() - time_msg.toSec();
    timestamp_debug_msg.ap20_msg_delta = imu->header.stamp.toSec() - time_msg.toSec();
    timestamp_debug_pub_.publish(timestamp_debug_msg);

    double new_line_delay = imu->header.stamp.toSec() - time_msg.toSec();
    if (line_delay_.size() == 10) {
        line_delay_.erase(line_delay_.begin());
        std::vector<double> sorted_delays = line_delay_;

        std::ostringstream oss;

        std::sort(sorted_delays.begin(), sorted_delays.end());
        double est = sorted_delays[sorted_delays.size() / 2];
        if (std::abs(est - new_line_delay) > 0.005) {
            ROS_ERROR_STREAM("Line delay is too high. Time delay: " << 
            new_line_delay << ". Deviates from recent delays by " << est - new_line_delay << "s.");
            soft_shutdown_ = true;
            return;

            // if (!backlog_cleared_) {
            //     ROS_ERROR("Backlog not cleared. CLEARING ONCE. <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
            //     backlog_cleared_ = true;
            //     line_delay_.clear();
            //     lookup_seq_.clear();
            //     lookup_ts_.clear();
            // } 
            // else {
            // }
            // return;
        }
    }

    lookup_seq_.push_back(imu->header.stamp.toSec());
    lookup_ts_.push_back(time_msg.toSec());


    line_delay_.push_back(new_line_delay);

    if (lookup_seq_.size() > 100) {
        lookup_seq_.erase(lookup_seq_.begin(), lookup_seq_.end() - 100);
        lookup_ts_.erase(lookup_ts_.begin(), lookup_ts_.end() - 100);
    }

    last_imu_message_header_time = imu->header.stamp.toSec();
    last_published_imu_seq_ = imu->header.seq;
    imu->header.stamp = time_msg;
    imu->header.frame_id = "ap20_imu";

    ap20_imu_pub_.publish(imu);

    

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
      runCommandAsync(start_imu_command);
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
        internal_counter_imu_ap20_valid_ = false;
        while (true) {
            changeImuMode(false);
            //read_timestamps();
            auto cleared_timestamps = timestamp_reader_->readTimestamps();
            ROS_INFO("Wait 0.5s and verify that no new timestamps are streaming");

            imu_message_queue_.clear();
            imu_message_counter_ = 0;
            timestamp_counter_ = 0;
            
            ros::Duration(0.5).sleep();

            // auto old_timestamps = read_timestamps();
            auto old_timestamps = timestamp_reader_->readTimestamps();
            ROS_INFO_STREAM("Timestamp queue size: " << old_timestamps.size());
            ROS_INFO_STREAM("IMU Message counter: " << imu_message_counter_);

            if (old_timestamps.empty() && imu_message_counter_ == 0) {
                break;
            }
            ROS_ERROR("Timestamps still streaming.");
        }

        {
            std::lock_guard<std::mutex> lock(imu_message_queue_mutex_);
            imu_message_queue_.clear();
            timestamp_queue_.clear();
            position_message_queue_.clear();
        }

        initializeVariables();
        int max_added = 0;
        lookup_seq_.clear();
        lookup_ts_.clear();
        line_delay_.clear();
        int steps_without_timestamps = 0;
        internal_counter_imu_ap20_valid_ = true;
        last_imu_message_header_time = -1;


        changeImuMode(true);

        ROS_INFO("Start normal operation");
        int k = 0;
        while (ros::ok() && !soft_shutdown_) {
            auto stamps = timestamp_reader_->readTimestamps();
            //auto stamps = read_timestamps();
            max_added = std::max(max_added, static_cast<int>(stamps.size()));
            if (stamps.empty()) {
                steps_without_timestamps++;
                if (steps_without_timestamps > 4000) {
                    ROS_ERROR("No timestamps for 4000 steps (8s). Shutting down.");
                    soft_shutdown_ = true;
                }
            } else{
                steps_without_timestamps = 0;
                for (const auto& ts : stamps) {
                    timestamp_queue_.push_front(ts);
                    ROS_INFO_STREAM("New Timestamp Message");
    
                    timestamp_counter_++;
                }
            }

            if (k<100){
                std::lock_guard<std::mutex> lock(imu_message_queue_mutex_);
                if (!imu_message_queue_.empty() && !timestamp_queue_.empty()) {
                    imu_message_queue_.pop_back();
                    timestamp_queue_.pop_back();
                    k++;
                }
                loop_rate.sleep();
                continue;
            }

            publishImuMessages();
            publishPositionMessages();
            loop_rate.sleep();
        }

        if (soft_shutdown_) {
            ROS_ERROR("Restarting AP20 due to line-delay error or gap in timestamps.");
        } else {
            ROS_ERROR("Shutting down due to unknown error.");
        }

    }

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ap20");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ROS_ERROR("Start untested cpp node.");

    AP20Node node(nh, private_nh);
    node.run();

    return 0;
}