#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <cmath>

// Add custom point type definition for Livox
struct LivoxPoint {
    PCL_ADD_POINT4D;           // Adds x,y,z,padding
    float intensity;           // intensity as float32
    uint8_t tag;              // tag as uint8
    uint8_t line;             // line as uint8
    double timestamp;         // timestamp as float64
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// Register the point struct with PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(LivoxPoint,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint8_t, tag, tag)
    (uint8_t, line, line)
    (double, timestamp, timestamp)
)



// Add helper function to calculate angle
float calculateAngleAroundZ(float x, float y) {
    float angle = atan2(y, x) * 180.0f / M_PI;
    if (angle < 0) {
        angle += 360.0f;
    }
    return angle;
}

void showProgress(const std::string& bag_name, int current, int total) {
    float percentage = (static_cast<float>(current) / total) * 100;
    std::cout << "\rProcessing " << bag_name << ": " << percentage << "% (" << current << "/" << total << ")" << std::flush;
}

int main(int argc, char** argv) {
    if (argc < 5) {
        ROS_ERROR("Usage: %s <input_rosbag1> <output_rosbag1> <input_rosbag2> <output_rosbag2>", argv[0]);
        return 1;
    }

    std::string input_bag1_path = argv[1];
    std::string output_bag1_path = argv[2];
    std::string input_bag2_path = argv[3];
    std::string output_bag2_path = argv[4];

    rosbag::Bag input_bag1, output_bag1, input_bag2, output_bag2;
    input_bag1.open(input_bag1_path, rosbag::bagmode::Read);
    output_bag1.open(output_bag1_path, rosbag::bagmode::Write);
    output_bag1.setCompression(rosbag::compression::LZ4);
    input_bag2.open(input_bag2_path, rosbag::bagmode::Read);
    output_bag2.open(output_bag2_path, rosbag::bagmode::Write);
    output_bag2.setCompression(rosbag::compression::LZ4);

    // Topics and parameters for Livox
    std::string input_topic1 = "/gt_box/livox/lidar";
    std::string output_topic1 = "/gt_box/livox/lidar_filtered";
    pcl::CropBox<pcl::PointXYZ> crop_box_filter1;
    crop_box_filter1.setMin(Eigen::Vector4f(-0.9, -0.15, -0.05, 1.0));
    crop_box_filter1.setMax(Eigen::Vector4f(0.2, 0.15, 0.3, 1.0));
    crop_box_filter1.setNegative(true);

    // Topics and parameters for Hesai
    std::string input_topic2 = "/gt_box/hesai/points";
    std::string output_topic2 = "/gt_box/hesai/points_filtered";
    pcl::CropBox<pcl::PointXYZ> crop_box_filter2;
    crop_box_filter2.setMin(Eigen::Vector4f(-0.2, -0.1, -0.15, 1.0));
    crop_box_filter2.setMax(Eigen::Vector4f(0.2, 0.65, 0.15, 1.0));
    crop_box_filter2.setNegative(true);

    // Process first bag
    rosbag::View view1(input_bag1);
    int total_msgs1 = view1.size();
    int processed_msgs1 = 0;

    for (const rosbag::MessageInstance& m : view1) {
        if (m.getTopic() == input_topic1) {
            sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (cloud_msg) {
                // In the first bag processing section, replace the point cloud declarations:
                pcl::PointCloud<LivoxPoint>::Ptr cloud(new pcl::PointCloud<LivoxPoint>());


                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
                pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
                pcl::PointCloud<pcl::PointXYZ>::Ptr angle_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());


                pcl::fromROSMsg(*cloud_msg, *cloud);
                
                
                for (const auto& point : cloud->points) {
                    if (point.tag == 0){
                        pcl::PointXYZ point_xyz;
                        point_xyz.x = point.x;
                        point_xyz.y = point.y;
                        point_xyz.z = point.z;
                        cloud_xyz->points.push_back(point_xyz);
                    }
                }

                crop_box_filter1.setInputCloud(cloud_xyz);
                crop_box_filter1.filter(*filtered_cloud);
                
                // Apply angle filter
                for (const auto& point : filtered_cloud->points) {
                    float angle = calculateAngleAroundZ(point.x, point.y);
                    if ((angle < 55) || (angle > 65.0f && angle < 140.0f) || (angle > 220.0f && angle < 295.0f) || (angle > 305.0f)){
                        angle_filtered_cloud->points.push_back(point);
                    }
                }
                angle_filtered_cloud->width = angle_filtered_cloud->points.size();
                angle_filtered_cloud->height = 1;


                sensor_msgs::PointCloud2 filtered_msg;
                pcl::toROSMsg(*angle_filtered_cloud, filtered_msg);
                filtered_msg.header = cloud_msg->header;

                output_bag1.write(output_topic1, m.getTime(), filtered_msg);
            }
        } else {
            output_bag1.write(m.getTopic(), m.getTime(), m);
        }
        showProgress("Bag 1", ++processed_msgs1, total_msgs1);
    }
    std::cout << std::endl;  // Finish the progress line for Bag 1.

    // Process second bag
    rosbag::View view2(input_bag2);
    int total_msgs2 = view2.size();
    int processed_msgs2 = 0;
    for (const rosbag::MessageInstance& m : view2) {
        if (m.getTopic() == input_topic2) {
            sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (cloud_msg) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
                pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());

                pcl::fromROSMsg(*cloud_msg, *cloud);
                crop_box_filter2.setInputCloud(cloud);
                crop_box_filter2.filter(*filtered_cloud);

                sensor_msgs::PointCloud2 filtered_msg;
                pcl::toROSMsg(*filtered_cloud, filtered_msg);
                filtered_msg.header = cloud_msg->header;

                output_bag2.write(output_topic2, m.getTime(), filtered_msg);
            }
        } else {
            output_bag2.write(m.getTopic(), m.getTime(), m);
        }
        showProgress("Bag 2", ++processed_msgs2, total_msgs2);
    }
    std::cout << std::endl;  // Finish the progress line for Bag 2.

    input_bag1.close();
    output_bag1.close();
    input_bag2.close();
    output_bag2.close();

    ROS_INFO("Filtered bags written to %s and %s", output_bag1_path.c_str(), output_bag2_path.c_str());
    return 0;
}
