#pragma once

#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <cmath>
#include <sl/Camera.hpp>
#include <string>

namespace zed_recording {
constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;

inline ros::Time sensorTime(const sl::Timestamp& timestamp) {
  ros::Time stamp;
  // SDK timestamps already use the host clock. A fixed 37-second GPS/UTC shift
  // must not be applied to these samples.
  stamp.fromNSec(timestamp.getNanoseconds());
  return stamp;
}

inline bool finite(const sl::float3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

template <class Covariance>
void copyCovariance(const sl::Matrix3f& input, double scale, Covariance& output) {
  output.fill(0.0);  // ROS: all zeros means covariance unknown; -1 means no measurement.
  for (int i = 0; i < 9; ++i)
    if (!std::isfinite(input.r[i]) || (i % 4 == 0 && input.r[i] < 0)) return;
  for (int i = 0; i < 9; ++i) output[i] = input.r[i] * scale;
}

inline sensor_msgs::Imu imuMessage(const sl::SensorsData::IMUData& data, const std::string& frame) {
  sensor_msgs::Imu msg;
  msg.header.stamp = sensorTime(data.timestamp);
  msg.header.frame_id = frame;
  msg.orientation_covariance[0] = -1.0;  // This topic intentionally contains inertial measurements only.
  msg.angular_velocity.x = data.angular_velocity.x * kDegreesToRadians;
  msg.angular_velocity.y = data.angular_velocity.y * kDegreesToRadians;
  msg.angular_velocity.z = data.angular_velocity.z * kDegreesToRadians;
  msg.linear_acceleration.x = data.linear_acceleration.x;
  msg.linear_acceleration.y = data.linear_acceleration.y;
  msg.linear_acceleration.z = data.linear_acceleration.z;
  copyCovariance(data.angular_velocity_covariance, kDegreesToRadians * kDegreesToRadians,
                 msg.angular_velocity_covariance);
  copyCovariance(data.linear_acceleration_covariance, 1.0, msg.linear_acceleration_covariance);
  return msg;
}

inline sensor_msgs::MagneticField magneticMessage(const sl::SensorsData::MagnetometerData& data,
                                                  const std::string& frame) {
  sensor_msgs::MagneticField msg;
  msg.header.stamp = sensorTime(data.timestamp);
  msg.header.frame_id = frame;
  msg.magnetic_field.x = data.magnetic_field_calibrated.x * 1e-6;
  msg.magnetic_field.y = data.magnetic_field_calibrated.y * 1e-6;
  msg.magnetic_field.z = data.magnetic_field_calibrated.z * 1e-6;
  return msg;  // Covariance unknown, not an invented sensor variance.
}

inline sensor_msgs::FluidPressure pressureMessage(const sl::SensorsData::BarometerData& data,
                                                  const std::string& frame) {
  sensor_msgs::FluidPressure msg;
  msg.header.stamp = sensorTime(data.timestamp);
  msg.header.frame_id = frame;
  msg.fluid_pressure = data.pressure * 100.0;  // SDK hPa -> ROS Pa.
  return msg;
}
}  // namespace zed_recording
