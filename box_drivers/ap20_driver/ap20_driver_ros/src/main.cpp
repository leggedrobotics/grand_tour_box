#include <fstream>
#include <deque>
#include <map>
#include <ros/ros.h>
#include <unistd.h>
#include <signal.h>
#include <ros/console.h>
#include <algorithm>

#include <thread>
#include <cstdlib>
#include <string>

#include <std_srvs/SetBool.h>
#include "std_msgs/Time.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/Imu.h"
#include "ap20_driver_ros/DebugInfo.h" // You'll need to create this custom message


// Function declarations
std::vector<ros::Time> read_timestamps();
void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
void position_callback(const geometry_msgs::PointStamped::ConstPtr& msg);

// Global variables (extern)
extern const std::string timestamper_module;
extern const std::string timestamps_fn;

extern std::deque<std_msgs::Time> timestamp_queue;
extern std::deque<sensor_msgs::ImuPtr> imu_message_queue;
extern std::deque<geometry_msgs::PointStampedPtr> position_message_queue;

// Mutexes
std::mutex imu_message_queue_mutex;

// Debug variables
bool debug_mode = true;
bool verbose = true;
std::deque<ros::Time> timestamp_arrival_time_queue;
std::deque<int> timestamp_arrival_counter_queue;
std::deque<ros::Time> imu_arrival_time_queue;
std::deque<int> imu_arrival_counter_queue;
std::deque<ros::Time> position_arrival_time_queue;
std::deque<int> position_arrival_counter_queue;

int imu_message_counter = 0;
int timestamp_counter = 0;
int position_message_counter = 0;

// Define global variables
const std::string timestamper_module = "time_stamper";
const std::string timestamps_fn = std::string("/sys/kernel/") + timestamper_module + "/ts_buffer";

std::deque<std_msgs::Time> timestamp_queue;
std::deque<sensor_msgs::ImuPtr> imu_message_queue;
std::deque<geometry_msgs::PointStampedPtr> position_message_queue;


std::deque<std_msgs::Time> timestamp_debug_queue;
std::deque<sensor_msgs::ImuPtr> imu_message_debug_queue;
std::deque<geometry_msgs::PointStampedPtr> position_message_debug_queue;
std::deque<std::pair<ros::Time, ros::Time>> imu_timestamp_mapping;


// Function to run the command asynchronously
void runCommandAsync(const std::string& command) {
    std::thread([command]() {
        system(command.c_str());
    }).detach();  // Detach the thread to make it non-blocking
}

template<typename T>
void trimQueueToSize(std::deque<T>& queue, size_t maxSize = 100) {
    if (queue.size() > maxSize) {
        queue.erase(queue.begin() + maxSize, queue.end());
    }
}

// Add this function to update the mapping
void updateImuTimestampMapping(const ros::Time& imu_time, const ros::Time& timestamp_time) {
    imu_timestamp_mapping.push_front(std::make_pair(imu_time, timestamp_time));
    if (imu_timestamp_mapping.size() > 100) {
        imu_timestamp_mapping.pop_back();
    }
}


bool change_imu_mode(ros::NodeHandle& nh, bool enable_streaming) {
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

bool change_imu_mode_ros(ros::NodeHandle& nh, bool enable_streaming) {
    ros::ServiceClient client = nh.serviceClient<std_srvs::SetBool>("/ap20/enable_streaming");
    
    // Wait for the service to be available with a 5-second timeout
    if (!client.waitForExistence(ros::Duration(5.0))) {
        ROS_ERROR("Service /ap20/enable_streaming is not available after waiting for 5 seconds");
        return false;
    }

    std_srvs::SetBool srv;
    srv.request.data = enable_streaming;

    if (client.call(srv)) {
        if (srv.response.success) {
            ROS_INFO("IMU mode changed successfully. Streaming is now %s.", enable_streaming ? "enabled" : "disabled");
            return true;
        } else {
            ROS_WARN("Failed to change IMU mode: %s", srv.response.message.c_str());
            return false;
        }
    } else {
        ROS_ERROR("Failed to call service /ap20/enable_streaming");
        return false;
    }
}


std::vector<ros::Time> read_timestamps()
{
  std::vector<ros::Time> ret {};

  std::ifstream timestamps_file(timestamps_fn);
  if (!timestamps_file.is_open()) {
    ROS_ERROR("Failed to open timestamps file: %s", timestamps_fn.c_str());
    return ret;
  }

  std::uint32_t secs, nsecs;
  char dot;
  while(timestamps_file >> secs >> dot >> nsecs)
  {
    if (dot != '.') {
      ROS_WARN("Unexpected format in timestamps file");
      continue;
    }
    ret.emplace_back(secs, nsecs);
  }
  return ret;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(imu_message_queue_mutex);
  imu_message_queue.push_front(boost::make_shared<sensor_msgs::Imu>(*msg));
  

  if (debug_mode) {
    if (verbose){ROS_INFO("Received imu message: %d", imu_message_counter);}
    // Check if this is not the first message
    if (!imu_message_debug_queue.empty()) {
      ros::Time last_timestamp = imu_message_debug_queue.front()->header.stamp;
      ros::Time current_timestamp = msg->header.stamp;
      
      // Calculate the time difference between the last and current message timestamps
      ros::Duration time_diff = current_timestamp - last_timestamp;

      // 6ms is the tolerance window (5ms + 20%)
      ros::Duration tolerance(0.006); 
      
      if (time_diff > tolerance) {
        ROS_ERROR("IMU message delay exceeded tolerance: %f ms", time_diff.toSec() * 1000.0);
      }
    }
    
    imu_message_debug_queue.push_front(boost::make_shared<sensor_msgs::Imu>(*msg));
    imu_arrival_time_queue.push_front(ros::Time::now());
    imu_arrival_counter_queue.push_front(imu_message_counter);
    imu_message_counter++;
  }
}

void position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  position_message_queue.push_front(boost::make_shared<geometry_msgs::PointStamped>(*msg));
  
  if (debug_mode) {
    position_message_debug_queue.push_front(boost::make_shared<geometry_msgs::PointStamped>(*msg));
    if (verbose){ROS_INFO("Received position message: %d", position_message_counter);}
    position_arrival_time_queue.push_front(ros::Time::now());
    position_arrival_counter_queue.push_front(imu_message_counter);
    position_message_counter++;
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ap20");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~"); // Create a private node handle


  // Get debug parameter
  private_nh.getParam("debug", debug_mode);
  private_nh.getParam("verbose", verbose);

  ros::Rate loop_rate(500);
  ros::Publisher ap20_imu_pub = private_nh.advertise<sensor_msgs::Imu>("imu", 200);
  ros::Publisher ap20_position_pub = private_nh.advertise<geometry_msgs::PointStamped>("prism_position", 100);
  ros::Publisher debug_pub = private_nh.advertise<ap20_driver_ros::DebugInfo>("debug", 10);

  ROS_INFO("Wait 10s for AP20 on device ROS driver to be ready");
  

  while (ros::ok())
  {
    change_imu_mode(n, false);
    // Wait for the network to transmit reaming messages
    auto old_timestamps = read_timestamps();
    ROS_INFO("Wait 1s and verify that no new timestamps are streaming");
    ros::Duration(1.0).sleep();
    auto old_timestamps_ = read_timestamps();
    
    // Wait for the service to be available with a 5-second timeout
    if (old_timestamps_.empty()) {
        break;
    }
    ROS_ERROR("Still timestamps are streaming.");
  }
  ROS_INFO("Driver successfully started");
  
  ros::Subscriber imu_sub = n.subscribe("/ap20/imu", 100, imu_callback);
  ros::Subscriber position_sub = n.subscribe("/ap20/tps", 20, position_callback);
  change_imu_mode(n, true);
  while (ros::ok())
  {
    // Read timestamps
    for (auto& ts : read_timestamps())
    {
      std_msgs::Time time;
      time.data = ts;
      timestamp_queue.push_front(time);
      

      if (debug_mode) {
        timestamp_debug_queue.push_front(time);
        //ROS_INFO("Received new timestamps: %d", timestamp_counter);
        timestamp_arrival_time_queue.push_front(ros::Time::now());
        timestamp_arrival_counter_queue.push_front(timestamp_counter);
        timestamp_counter++;
      }
    }

      // Debug publish full state
    if (debug_mode) {
        ap20_driver_ros::DebugInfo debug_msg;

        // Helper function to safely get the first N elements from a deque
        auto getFirstN = [](const auto& queue, size_t n) {
            return std::vector<typename std::decay<decltype(queue)>::type::value_type>(
                queue.begin(), queue.begin() + std::min(queue.size(), n));
        };

        // Convert std_msgs::Time to ros::Time
        auto convertStdMsgTime = [](const std_msgs::Time& t) {
            return t.data;
        };

        // Fill timestamp_arrival_times
        for (const auto& t : getFirstN(timestamp_arrival_time_queue, 10)) {
            debug_msg.timestamp_arrival_times.push_back(t);
        }

        // Fill timestamp_arrival_counters
        debug_msg.timestamp_arrival_counters = getFirstN(timestamp_arrival_counter_queue, 10);

        // Fill timestamps
        for (const auto& t : getFirstN(timestamp_queue, 10)) {
            debug_msg.timestamps.push_back(convertStdMsgTime(t));
        }

        // Fill position_arrival_times
        for (const auto& t : getFirstN(position_arrival_time_queue, 10)) {
            debug_msg.position_arrival_times.push_back(t);
        }

        // Fill position_arrival_counters
        debug_msg.position_arrival_counters = getFirstN(position_arrival_counter_queue, 10);

        // Fill imu_arrival_times
        for (const auto& t : getFirstN(imu_arrival_time_queue, 10)) {
            debug_msg.imu_arrival_times.push_back(t);
        }

        // Fill imu_arrival_counters
        debug_msg.imu_arrival_counters = getFirstN(imu_arrival_counter_queue, 10);

        // Fill position_timestamps and imu_timestamps
        auto position_timestamps = getFirstN(position_message_debug_queue, 10);
        auto imu_timestamps = getFirstN(imu_message_debug_queue, 10);

        for (const auto& pos : position_timestamps) {
            debug_msg.position_timestamps.push_back(pos->header.stamp);
        }
        for (const auto& imu : imu_timestamps) {
            debug_msg.imu_timestamps.push_back(imu->header.stamp);
        }
        debug_pub.publish(debug_msg);


        // TODO remove queue elements more then 10 starting from the end for
        trimQueueToSize(timestamp_debug_queue);
        trimQueueToSize(imu_message_debug_queue);
        trimQueueToSize(position_message_debug_queue);
    }

    // Pubilish position messages with corrected timestamps
    if (!position_message_queue.empty() && !imu_timestamp_mapping.empty()) {
        geometry_msgs::PointStampedPtr position = position_message_queue.back();
        position_message_queue.pop_back();

        // Find the closest IMU timestamp in the mapping
        ros::Time closest_imu_time;
        ros::Time closest_timestamp_time;
        double min_diff = std::numeric_limits<double>::max();

        for (const auto& mapping : imu_timestamp_mapping) {
            double time_diff = std::abs((mapping.first - position->header.stamp).toSec());
            if (time_diff < min_diff) {
                min_diff = time_diff;
                closest_imu_time = mapping.first;
                closest_timestamp_time = mapping.second;
            }
        }

        // Calculate the time difference and apply it to the position timestamp
        if (!imu_timestamp_mapping.empty()) {
            ros::Duration time_diff = position->header.stamp - closest_imu_time;
            ros::Time original_position_time = position->header.stamp;
            position->header.stamp = closest_timestamp_time + time_diff;
            
            if (verbose){
              // Prepare formatted output
              std::stringstream ss;
              ss << std::fixed << std::setprecision(9);  // Set precision for nanoseconds

              ss << "\n=== Totalstation Measurement Precision ===\n";
              ss << "Original Position Time: " << original_position_time.sec << "." << std::setfill('0') << std::setw(9) << original_position_time.nsec << "\n";
              ss << "Closest IMU Time:       " << closest_imu_time.sec << "." << std::setfill('0') << std::setw(9) << closest_imu_time.nsec << "\n";
              ss << "Closest Timestamp:      " << closest_timestamp_time.sec << "." << std::setfill('0') << std::setw(9) << closest_timestamp_time.nsec << "\n";
              ss << "Time Difference:        " << std::setprecision(9) << time_diff.toSec() << " seconds\n";
              ss << "Corrected Position Time:" << position->header.stamp.sec << "." << std::setfill('0') << std::setw(9) << position->header.stamp.nsec << "\n";
              
              // Calculate and log time differences in milliseconds
              double diff_imu_orig = (closest_imu_time - original_position_time).toSec() * 1000.0;
              
              ss << std::fixed << std::setprecision(3);  // Set precision for milliseconds
              ss << "Diff (IMU - Original):     " << diff_imu_orig << " ms\n";
              ss << "==========================================\n";

              ROS_INFO_STREAM(ss.str());
            }
        }
        ap20_position_pub.publish(*position);
    }

    while (!timestamp_queue.empty() && !imu_message_queue.empty()) {
      std_msgs::Time time;
      sensor_msgs::ImuPtr imu;
      {
          std::lock_guard<std::mutex> lock_imu(imu_message_queue_mutex);
          time = timestamp_queue.back();
          timestamp_queue.pop_back();
          imu = imu_message_queue.back();
          imu_message_queue.pop_back();
      }

      updateImuTimestampMapping(imu->header.stamp, time.data);
      imu->header.stamp = time.data;
      ap20_imu_pub.publish(*imu);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  change_imu_mode(n, false);

  
  return 0;
}