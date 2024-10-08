#include <ros/ros.h>
#include "detector_node.h"

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "GTCameraDetectionNode");

    // Create a ROS NodeHandle
    ros::NodeHandle nh;

    // Instantiate the node
    CameraDetectorNode node(nh);

    // Keep the node alive (if needed, for example for subscriber/publisher loops)
    ros::spin();

    return 0;
}
