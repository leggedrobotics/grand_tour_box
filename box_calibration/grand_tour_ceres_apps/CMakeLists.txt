cmake_minimum_required(VERSION 3.16)
project(grand_tour_ceres_apps)

set(CMAKE_CXX_STANDARD 20)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        grand_tour_camera_detection_msgs
        roslib
        rosbag
        sensor_msgs
        ap20_driver_ros
)
find_package(Boost 1.70 REQUIRED COMPONENTS graph)
find_package(Ceres 2.2 REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)


add_library(gtboxcalibration
        include/gtboxcalibration/argparse.h
        include/gtboxcalibration/argparsers.h src/argparsers.cpp
        include/gtboxcalibration/camerageometry.h
        include/gtboxcalibration/ceresprograms.h src/ceresprogram.cpp
        include/gtboxcalibration/ceresproblems.h src/ceresproblems.cpp
        include/gtboxcalibration/datatypes.h
        include/gtboxcalibration/detectiongraphutils.h src/detectiongraphutils.cpp
        include/gtboxcalibration/interpolation3d.h src/intepolation_3_d.cpp
        include/gtboxcalibration/json.h
        include/gtboxcalibration/parameterhelpers.h
        include/gtboxcalibration/prismpositionresiduals.h src/prismpositionresiduals.cpp
        include/gtboxcalibration/reprojectionresiduals.h src/reprojectionresiduals.cpp
        include/gtboxcalibration/utils.h src/utils.cpp
        include/gtboxcalibration/voxel_2d.h src/voxel_2d.cpp
)
target_include_directories(gtboxcalibration PUBLIC include)
target_link_libraries(gtboxcalibration PUBLIC Eigen3::Eigen ${OpenCV_LIBS} yaml-cpp Ceres::ceres)

catkin_package(
        INCLUDE_DIRS
        LIBRARIES gtboxcalibration
        CATKIN_DEPENDS roscpp std_msgs grand_tour_camera_detection_msgs
)

add_executable(compute_connectivity connectivity.cpp)
target_link_libraries(compute_connectivity Eigen3::Eigen ${Boost_LIBRARIES} gtboxcalibration)

add_executable(camera_camera_bundle_adjustment camera_camera_bundle_adjustment.cpp)
target_link_libraries(camera_camera_bundle_adjustment PRIVATE
        ${OpenCV_LIBS}
        Ceres::ceres Eigen3::Eigen gtboxcalibration)

add_executable(cameras_prism_bundle_adjustment cameras_prism_bundle_adjustment.cpp)
target_link_libraries(cameras_prism_bundle_adjustment PRIVATE
        gtboxcalibration
        Ceres::ceres Eigen3::Eigen)

add_executable(camera_camera_online_calibration ros_camera_camera_online_calibration.cpp
        ros_camera_camera_program.cpp
        ros_camera_camera_online_program.cpp
        ros_camera_camera_parser.cpp
        ros_utils.cpp
)
target_link_libraries(camera_camera_online_calibration gtboxcalibration ${catkin_LIBRARIES}
)
target_include_directories(camera_camera_online_calibration PRIVATE ${catkin_INCLUDE_DIRS})

add_executable(camera_camera_offline_calibration ros_camera_camera_offline_calibration.cpp
        ros_camera_camera_program.cpp
        ros_camera_camera_offline_program.cpp
        ros_camera_camera_parser.cpp
        ros_utils.cpp
)
target_link_libraries(camera_camera_offline_calibration gtboxcalibration ${catkin_LIBRARIES}
)
target_include_directories(camera_camera_offline_calibration PRIVATE ${catkin_INCLUDE_DIRS})

add_executable(camera_prism_offline_calibration ros_camera_prism_offline_calibration.cpp
        ros_camera_prism_parser.cpp
        ros_camera_prism_program.cpp
        ros_utils.cpp
)
target_link_libraries(camera_prism_offline_calibration gtboxcalibration ${catkin_LIBRARIES}
)
target_include_directories(camera_prism_offline_calibration PRIVATE ${catkin_INCLUDE_DIRS})
