#include <fstream>

#include <signal.h>
#include <queue>
#include <ros/ros.h>
#include <ros/console.h>
#include "std_msgs/Time.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/Imu.h"

std::mutex timestamp_imu_queue_mutex;
std::mutex position_queue_mutex;

// The name of the kernel timestamping module reading from our loopback GPIO (26)
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
  const std::lock_guard<std::mutex> lock(timestamp_imu_queue_mutex);
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
  const std::lock_guard<std::mutex> lock(timestamp_imu_queue_mutex);
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
  const std::lock_guard<std::mutex> lock(position_queue_mutex);
  position_message_queue.push(msg);
}

int main(int argc, char **argv)
{


  ros::init(argc, argv, "ap20_trigger");
  ros::NodeHandle n;

  ros::Publisher ap20_imu_pub = n.advertise<sensor_msgs::Imu>("ap20_imu", 1000);
  ros::Publisher ap20_position_pub = n.advertise<geometry_msgs::PointStamped>("ap20_prism_position", 100);

  ros::Subscriber imu_sub = n.subscribe("/imu/data_raw", 1000, imu_callback);
  ros::Subscriber position_sub = n.subscribe("/leica/position", 1000, position_callback);
  //ros::Rate loop_rate(pwm_freq);
 



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
      // TODO: Throw away message if time smaller than smallest time in queue
      geometry_msgs::PointStampedPtr position = position_message_queue.front();
      ros::Time position_time = position->header.stamp;
      position_message_queue.pop();

      ros::Duration offset(0, 4000000);
      auto lower_it = time_matcher.lower_bound(position_time - offset);
      auto upper_it = time_matcher.upper_bound(position_time + offset);
      int num_relevant_timestamps = std::distance(lower_it, upper_it);
      if (num_relevant_timestamps == 1){
        position->header.stamp = lower_it->second;
      }
      else if (num_relevant_timestamps == 2){
        ros::Time lower_timestamp_ap20 = lower_it->first;
        ros::Time lower_timestamp_jetson = lower_it->second;
        lower_it++;
        ros::Time higher_timestamp_ap20 = lower_it->first;
        ros::Time higher_timestamp_jetson = lower_it->second;
        // TODO: interpolate between to get jetson timestamp of position
        // position->header.stamp = interpolated_time;
        ros::Duration ap20_duration = higher_timestamp_ap20 - lower_timestamp_ap20;
        ros::Duration jetson_duration = higher_timestamp_jetson - lower_timestamp_jetson;
        ros::Duration position_duration_ap20 = position_time - lower_timestamp_ap20;
        float frac = position_duration_ap20.toSec() / ap20_duration.toSec();

        position->header.stamp = lower_timestamp_jetson + ros::Duration(frac*jetson_duration.toSec());
      }
      else{
        ROS_ERROR("Zero or too many relevant timestamps");
      }
      ap20_position_pub.publish(*position);

      // delete all keys in the map smaller than position time
      for(auto it = time_matcher.begin(); it != time_matcher.lower_bound(position_time); ){
        it = time_matcher.erase(it);
      }
    }
    ros::spinOnce();
    //loop_rate.sleep();
  }

  return 0;
}
