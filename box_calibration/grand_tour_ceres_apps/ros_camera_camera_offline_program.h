//
// Created by fu on 27/09/24.
//

#ifndef GRAND_TOUR_CERES_APPS_ROS_CAMERA_CAMERA_ONLINE_PROGRAM_H
#define GRAND_TOUR_CERES_APPS_ROS_CAMERA_CAMERA_ONLINE_PROGRAM_H

#include <gtboxcalibration/ceresprograms.h>
#include <ros/ros.h>
#include <grand_tour_camera_detection_msgs/CameraDetections.h>
#include <grand_tour_camera_detection_msgs/StopOptimizingService.h>
#include <grand_tour_camera_detection_msgs/StartRecordingCalibrationDataService.h>
#include "ros_camera_camera_parser.h"
#include "gtboxcalibration/voxel_2d.h"
#include "ros_camera_camera_program.h"
#include <ros/callback_queue.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

struct ROSCameraCameraOfflineProgram : public ROSCameraCameraProgram {
    explicit ROSCameraCameraOfflineProgram(ROSCameraCameraParser);

    void run();

    bool loadRosbagsIntoProgram();

protected:
    bool publishDetectionsUsed(const grand_tour_camera_detection_msgs::CameraDetections &camera_detections) override;

    bool is_valid = false;
    std::vector<std::string> bag_paths;

};


#endif //GRAND_TOUR_CERES_APPS_ROS_CAMERA_CAMERA_ONLINE_PROGRAM_H
