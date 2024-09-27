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


std::deque<ros::Time> timestamp_queue_;
std::deque<sensor_msgs::Imu> imu_message_queue_;
std::deque<geometry_msgs::PointStamped> position_message_queue_;
std::vector<double> lookup_seq_;
std::vector<double> lookup_ts_;
std::vector<double> line_delay_;

bool publish_timestamps_;
int imu_message_counter_ = 0;
int timestamp_counter_ = 0;
int internal_counter_imu_ap20_ = -1;
bool shutdown_ = false;
int last_published_imu_seq_ = -1;

ros::init(argc, argv, "ap20");
ros::NodeHandle n;
ros::NodeHandle private_nh("~");
ros::NodeHandle nh_;
ros::Publisher ap20_imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 200);
ros::Publisher ap20_position_pub_ = nh_.advertise<geometry_msgs::PointStamped>("prism_position", 100);

std::unique_ptr<ContinuousTimestampReader> timestamp_reader_  = std::make_unique<ContinuousTimestampReader>("/sys/kernel/time_stamper/ts_buffer");

ros::Subscriber imu_sub_ = nh_.subscribe("/ap20/imu", 100, &AP20Node::imuCallback, this);
ros::Subscriber position_sub_ = nh_.subscribe("/ap20/tps", 20, &AP20Node::positionCallback, this);


class ContinuousTimestampReader {
public:
    ContinuousTimestampReader(const std::string& timestamps_fn)
        : timestamps_fn_(timestamps_fn) {
        openFile();
    }

    void openFile() {
        if (!timestamps_file_.is_open()) {
            timestamps_file_.open(timestamps_fn_);
            if (!timestamps_file_.is_open()) {
                ROS_ERROR_STREAM("Failed to open timestamps file: " << timestamps_fn_);
            }
        }
    }

    std::vector<ros::Time> readTimestamps() {
        std::vector<ros::Time> ret;
        if (!timestamps_file_.is_open()) {
            openFile();
            if (!timestamps_file_.is_open()) {
                return ret;
            }
        }

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

private:
    std::string timestamps_fn_;
    std::ifstream timestamps_file_;
};


// Function to run the command asynchronously
void runCommandAsync(const std::string& command) {
    std::thread([command]() {
        system(command.c_str());
    }).detach();  // Detach the thread to make it non-blocking
}

bool changeImuMode(ros::NodeHandle& nh, bool enable_streaming) {
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

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    ROS_INFO("IMU");

    imu_message_queue_.push_front(*msg);

    if (msg->header.seq != internal_counter_imu_ap20_ + 1 && internal_counter_imu_ap20_ != -1) {
        ROS_ERROR_STREAM("Missed message: " << internal_counter_imu_ap20_ << " -> " << msg->header.seq);
        shutdown_ = true;
    }

    internal_counter_imu_ap20_ = msg->header.seq;
    imu_message_counter_++;
}

void positionCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    ROS_INFO("Position");
    position_message_queue_.push_front(*msg);
}

void publishPositionMessages() {
    if (!position_message_queue_.empty()) {
        auto position = position_message_queue_.back();
        position_message_queue_.pop_back();

        double ap20_original_ts = position.header.stamp.toSec();

        if (lookup_seq_.empty()) {
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
            return;
        }

        if (j == 0) {
            return;
        }

        if (j == lookup_ts_.size()) {
            position_message_queue_.push_front(position);
            return;
        }

        double rate = (ap20_original_ts - lookup_seq_[j - 1]) / (lookup_seq_[j] - lookup_seq_[j - 1]);
        double new_ts = lookup_ts_[j - 1] + ((lookup_ts_[j] - lookup_ts_[j - 1]) * rate);

        position.header.stamp = ros::Time(new_ts);
        ap20_position_pub_.publish(position);
        lookup_seq_.erase(lookup_seq_.begin(), lookup_seq_.begin() + j - 1);
        lookup_ts_.erase(lookup_ts_.begin(), lookup_ts_.begin() + j - 1);
    }
}

void publishImuMessages() {
    if (!timestamp_queue_.empty() && !imu_message_queue_.empty()) {
        auto time_msg = timestamp_queue_.back();
        timestamp_queue_.pop_back();
        auto imu = imu_message_queue_.back();
        imu_message_queue_.pop_back();

        lookup_seq_.push_back(imu.header.stamp.toSec());
        lookup_ts_.push_back(time_msg.toSec());

        double new_line_delay = lookup_seq_.back() - lookup_ts_.back();
        if (line_delay_.size() == 5) {
            line_delay_.erase(line_delay_.begin());
            std::vector<double> sorted_delays = line_delay_;
            std::sort(sorted_delays.begin(), sorted_delays.end());
            double est = sorted_delays[sorted_delays.size() / 2];
            if (std::abs(est - new_line_delay) > 0.005) {
                ROS_ERROR_STREAM("Line delay is too high. Time skip detected: " << new_line_delay);
                shutdown_ = true;
            }
        }

        line_delay_.push_back(new_line_delay);

        if (lookup_seq_.size() > 100) {
            lookup_seq_.erase(lookup_seq_.begin(), lookup_seq_.end() - 100);
            lookup_ts_.erase(lookup_ts_.begin(), lookup_ts_.end() - 100);
        }

        last_published_imu_seq_ = imu.header.seq;
        imu.header.stamp = time_msg;
        imu.header.frame_id = "ap20_imu";
        ap20_imu_pub_.publish(imu);
    }
}


int main(int argc, char **argv)
{
    ros::Rate rate(500);
    while (ros::ok()) {
        while (true) {
            changeImuMode(nh_, true);
            timestamp_reader_->readTimestamps();
            ROS_INFO("Wait 0.5s and verify that no new timestamps are streaming");

            imu_message_queue_.clear();
            imu_message_counter_ = 0;
            timestamp_counter_ = 0;
            internal_counter_imu_ap20_ = -1;
            ros::Duration(1.0).sleep();

            auto old_timestamps_ = timestamp_reader_->readTimestamps();

            if (old_timestamps_.empty() && imu_message_counter_ == 0) {
                break;
            }
            ROS_ERROR("Still timestamps are streaming.");
        }

        changeImuMode(nh_, true);

        int max_added = 0;
        shutdown_ = false;
        position_message_queue_.clear();
        lookup_seq_.clear();
        lookup_ts_.clear();
        line_delay_.clear();
        last_published_imu_seq_ = -1;

        while (ros::ok() && !shutdown_) {
            auto stamps = timestamp_reader_->readTimestamps();
            max_added = std::max(max_added, static_cast<int>(stamps.size()));
            for (const auto& ts : stamps) {
                timestamp_queue_.push_front(ts);
                timestamp_counter_++;
            }

            publishImuMessages();
            publishPositionMessages();
            ros::spinOnce();
            rate.sleep();
        }

        if (shutdown_) {
            ROS_ERROR("Shutting down due to line-delay error.");
            break;
        }
    }
}