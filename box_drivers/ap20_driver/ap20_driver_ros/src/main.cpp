#include <fstream>

#include <signal.h>
#include <queue>
#include <ros/ros.h>
#include <ros/console.h>
#include "std_msgs/Time.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/Imu.h"

// the name of the kernel timestamping module reading from the triggered gpio 325
const std::string timestamper_module = "time_stamper";
const std::string timestamps_fn = std::string("/sys/kernel/") + timestamper_module + "/ts_buffer";

std::queue<std_msgs::Time> timestamp_queue;
std::queue<sensor_msgs::ImuPtr> imu_message_queue;
std::queue<sensor_msgs::ImuPtr> timecorrected_imu_message_queue;
std::queue<geometry_msgs::PointStampedPtr> position_message_queue;
std::map<ros::Time, ros::Time> time_matcher;

std::vector<ros::Time> read_timestamps()
{
  std::vector<ros::Time> ret {};

  std::ifstream timestamps_file(timestamps_fn);
  int secs, nsecs;
  char dot;
  while(timestamps_file >> secs >> dot >> nsecs)
  {
    ret.emplace_back(secs, nsecs);
  }
  return ret;
}

void timestamp_callback(const std_msgs::Time& msg)
{
  if(imu_message_queue.empty()){
    timestamp_queue.push(msg);
  }
  else{
    auto imu = imu_message_queue.front();
    imu_message_queue.pop();
    time_matcher.insert({imu->header.stamp, msg.data});
    imu->header.stamp = msg.data;
    timecorrected_imu_message_queue.push(imu);
  }  
}

void imu_callback(const sensor_msgs::Imu::Ptr& msg)
{
  if(timestamp_queue.empty()){
    imu_message_queue.push(msg);
  }
  else{
    auto time = timestamp_queue.front();
    timestamp_queue.pop();
    time_matcher.insert({msg->header.stamp, time.data});
    msg->header.stamp = time.data;
    timecorrected_imu_message_queue.push(msg);
  }
}

void position_callback(const geometry_msgs::PointStamped::Ptr& msg)
{
  position_message_queue.push(msg);
}

int main(int argc, char **argv)
{
  // empty old timestamps from previous measurements
  while(!read_timestamps().empty()) {}

  ros::init(argc, argv, "ap20_trigger");
  ros::NodeHandle n;

  ros::Publisher ap20_imu_pub = n.advertise<sensor_msgs::Imu>("~imu", 1000);
  ros::Publisher ap20_position_pub = n.advertise<geometry_msgs::PointStamped>("~prism_position", 100);

  ros::Subscriber imu_sub = n.subscribe("/box_gt/ap20/imu/data_raw", 1000, imu_callback);
  ros::Subscriber position_sub = n.subscribe("/box_gt/ap20/leica/position", 1000, position_callback);
  ros::Rate loop_rate(200);

  while (ros::ok())
  {
    for (auto& ts : read_timestamps())
    {
      std_msgs::Time time;
      time.data = ts;
      timestamp_callback(time);
    }

    while(!timecorrected_imu_message_queue.empty()){
      ap20_imu_pub.publish(timecorrected_imu_message_queue.front());
      timecorrected_imu_message_queue.pop();
    }

    if(!position_message_queue.empty()){
      geometry_msgs::PointStampedPtr position = position_message_queue.front();
      ros::Time ap20_position = position->header.stamp;
      position_message_queue.pop();

      // ap20 timestamps are every 5ms, so ns = xx0'000'000 or = xx5'000'000
      // an offset of +-4ms achieves one measurement >= ap20_position
      // and one measurement <= ap20_position (can be the same measurement)
      ros::Duration offset(0, 4000000);
      auto lower_it = time_matcher.lower_bound(ap20_position - offset);
      auto upper_it = time_matcher.upper_bound(ap20_position + offset);
      int num_relevant_timestamps = std::distance(lower_it, upper_it);
      if (num_relevant_timestamps == 1){
        position->header.stamp = lower_it->second;
      }
      else if (num_relevant_timestamps == 2){
        // interpolate between two imu timestamps a & b
        // ap20_a + frac*(ap20_b - ap20_a) = ap20_position
        // jetson_position = jetson_a + frac * (jetson_b - jetson_a)
        ros::Time ap20_a = lower_it->first;
        ros::Time jetson_a = lower_it->second;
        lower_it++;
        ros::Time ap20_b = lower_it->first;
        ros::Time jetson_b = lower_it->second;

        double frac = (ap20_position - ap20_a).toSec() / (ap20_b - ap20_a).toSec();
        position->header.stamp = jetson_a + ros::Duration(frac * (jetson_b - jetson_a).toSec());        
      }
      else{
        ROS_ERROR("Zero or too many relevant timestamps");
      }
      ap20_position_pub.publish(*position);

      // delete all the imu measurements which happened before the current position measurement
      for(auto it = time_matcher.begin(); it != time_matcher.lower_bound(ap20_position); ){
        it = time_matcher.erase(it);
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
