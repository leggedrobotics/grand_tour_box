#include <fstream>
#include <deque>
#include <map>
#include <ros/ros.h>
#include "std_msgs/Time.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/Imu.h"
#include <unistd.h>
#include <signal.h>
#include <ros/console.h>

// Function declarations
std::vector<ros::Time> read_timestamps();
void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
void position_callback(const geometry_msgs::PointStamped::ConstPtr& msg);

// Global variables (extern)
extern const std::string timestamper_module;
extern const std::string timestamps_fn;

extern std::deque<std_msgs::Time> timestamp_queue;
extern std::deque<sensor_msgs::ImuPtr> imu_message_queue;
extern std::deque<sensor_msgs::ImuPtr> timecorrected_imu_message_queue;
extern std::deque<geometry_msgs::PointStampedPtr> position_message_queue;
extern std::map<ros::Time, ros::Time> time_matcher;



// Define global variables
const std::string timestamper_module = "time_stamper";
const std::string timestamps_fn = std::string("/sys/kernel/") + timestamper_module + "/ts_buffer";

std::deque<std_msgs::Time> timestamp_queue;
std::deque<sensor_msgs::ImuPtr> imu_message_queue;
std::deque<sensor_msgs::ImuPtr> timecorrected_imu_message_queue;
std::deque<geometry_msgs::PointStampedPtr> position_message_queue;
std::map<ros::Time, ros::Time> time_matcher;

std::vector<ros::Time> read_timestamps()
{
  std::vector<ros::Time> ret {};

  std::ifstream timestamps_file(timestamps_fn);
  if (!timestamps_file.is_open()) {
    ROS_ERROR("Failed to open timestamps file: %s", timestamps_fn.c_str());
    return ret;
  }

  int secs, nsecs;
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
  imu_message_queue.push_front(boost::make_shared<sensor_msgs::Imu>(*msg));
}

void position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  position_message_queue.push_front(boost::make_shared<geometry_msgs::PointStamped>(*msg));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ap20_timecorrection");
  ros::NodeHandle n;

  ros::Publisher ap20_imu_pub = n.advertise<sensor_msgs::Imu>("/gt_box/ap20/imu", 200);
  ros::Publisher ap20_position_pub = n.advertise<geometry_msgs::PointStamped>("/gt_box/ap20/prism_position", 100);

  ros::Duration(5.0).sleep(); // Use ros::Duration instead of sleep()

  std::string stop_imu_command = "rosservice call /ap20/enable_streaming \"{data: false}\"";
  int result = system(stop_imu_command.c_str());
  if (result == 0) {
    ROS_INFO("IMU stopped successfully.");
  } else {
    ROS_ERROR("Stopping IMU failed, execution failed with code %d", result);
  }

  ros::Duration(1.0).sleep();

  ros::Subscriber imu_sub = n.subscribe("/ap20/imu", 200, imu_callback);
  ros::Subscriber position_sub = n.subscribe("/ap20/tps", 100, position_callback);
  ros::Rate loop_rate(500);

  auto old_timestamps = read_timestamps();

  ros::Duration(3).sleep();

  std::string start_imu_command = "rosservice call /ap20/enable_streaming \"{data: true}\"";
  int result_start = system(start_imu_command.c_str());
  if (result_start == 0) {
    ROS_INFO("IMU started successfully.");
  } else {
    ROS_ERROR("Starting IMU failed, execution failed with code %d", result_start);
  }

  int counter_imu = 0;
  int counter_timestamp = 0;
  while (ros::ok())
  {
    for (auto& ts : read_timestamps())
    {
      std_msgs::Time time;
      time.data = ts;
      timestamp_queue.push_front(time);
    }

    while (!timestamp_queue.empty() && !imu_message_queue.empty()) {
      std_msgs::Time time = timestamp_queue.back();
      timestamp_queue.pop_back();
      sensor_msgs::ImuPtr imu = imu_message_queue.back();
      imu_message_queue.pop_back();
      imu->header.stamp = time.data;
      counter_imu++;
      counter_timestamp++;
      ap20_imu_pub.publish(*imu);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}