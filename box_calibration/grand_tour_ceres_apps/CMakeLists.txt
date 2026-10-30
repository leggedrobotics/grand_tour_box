cmake_minimum_required(VERSION 3.16)
project(grand_tour_ceres_apps)

# ROS Noetic message serialization still uses allocator APIs removed in C++20.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        grand_tour_camera_detection_msgs
        roslib
        rosbag
        sensor_msgs
        geometry_msgs
        cv_bridge
        ap20_driver_ros
        grand_tour_camera_detectors
)
find_package(Boost 1.70 REQUIRED COMPONENTS graph)
find_package(Ceres 2.2 REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

include(FetchContent)
FetchContent_Declare(rerun_sdk URL
        https://github.com/rerun-io/rerun/releases/download/0.30.0/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

add_subdirectory(Sophus)

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
        include/gtboxcalibration/imuresiduals.h src/imuresiduals.cpp
)
target_include_directories(gtboxcalibration PUBLIC include)
target_link_libraries(gtboxcalibration PUBLIC Eigen3::Eigen ${OpenCV_LIBS} yaml-cpp Ceres::ceres Sophus::Sophus)

catkin_package(
        INCLUDE_DIRS include
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
target_link_libraries(cameras_prism_bundle_adjustment PRIVATE gtboxcalibration Ceres::ceres Eigen3::Eigen)

add_executable(camera_camera_online_calibration ros_camera_camera_online_calibration.cpp
        ros_camera_camera_program.cpp
        ros_camera_camera_online_program.cpp
        ros_camera_camera_parser.cpp
        ros_utils.cpp
        camera_camera_viz_interface.cpp
        rerun_camera_camera_viz.cpp
)
target_link_libraries(camera_camera_online_calibration gtboxcalibration ${catkin_LIBRARIES} rerun_sdk
)
target_include_directories(camera_camera_online_calibration PRIVATE ${catkin_INCLUDE_DIRS})

add_executable(camera_camera_offline_calibration ros_camera_camera_offline_calibration.cpp
        ros_camera_camera_program.cpp
        ros_camera_camera_offline_program.cpp
        ros_camera_camera_parser.cpp
        ros_utils.cpp
        camera_camera_viz_interface.cpp
        rerun_camera_camera_viz.cpp
)
target_link_libraries(camera_camera_offline_calibration gtboxcalibration ${catkin_LIBRARIES} rerun_sdk
)
target_include_directories(camera_camera_offline_calibration PRIVATE ${catkin_INCLUDE_DIRS})

add_executable(camera_prism_offline_calibration ros_camera_prism_offline_calibration.cpp
        ros_camera_prism_program.cpp
        ros_camera_prism_parser.cpp
        rerun_camera_prism_viz.cpp
        ros_utils.cpp)
target_link_libraries(camera_prism_offline_calibration PRIVATE
        gtboxcalibration
        Ceres::ceres Eigen3::Eigen
        ${catkin_LIBRARIES} rerun_sdk)
target_include_directories(camera_prism_offline_calibration PRIVATE ${catkin_INCLUDE_DIRS})

add_executable(camera_imu_offline_calibration
        ros_camera_imu_offline_calibration.cpp
        ros_camera_imu_parser.cpp ros_camera_imu_program.cpp
        imu_integrator.cpp
        rerun_camera_imu_viz.cpp
        ros_utils.cpp)

target_include_directories(
        camera_imu_offline_calibration
        PRIVATE ${catkin_INCLUDE_DIRS}
)

target_link_libraries(camera_imu_offline_calibration PRIVATE
        gtboxcalibration
        Ceres::ceres Eigen3::Eigen
        ${catkin_LIBRARIES} rerun_sdk
)

add_executable(camera_imu_preprocess ros_camera_imu_preprocess.cpp)
target_include_directories(camera_imu_preprocess PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(camera_imu_preprocess PRIVATE
        gtboxcalibration
        Eigen3::Eigen
        ${catkin_LIBRARIES}
)

add_executable(camera_imu_detection_benchmark ros_camera_imu_detection_benchmark.cpp)
target_include_directories(camera_imu_detection_benchmark PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(camera_imu_detection_benchmark PRIVATE
        gtboxcalibration
        Eigen3::Eigen
        ${catkin_LIBRARIES}
)

if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_prism_correctness tests/test_prism_correctness.cpp)
    target_link_libraries(test_prism_correctness gtboxcalibration)
    catkin_add_gtest(test_utils tests/test_utils.cpp)
    target_link_libraries(test_utils gtboxcalibration yaml-cpp)

    catkin_add_gtest(test_interpolation3d tests/test_interpolation3d.cpp)
    target_link_libraries(test_interpolation3d gtboxcalibration gtest_main)

    catkin_add_gtest(test_imu_correctness tests/test_imu_correctness.cpp)
    target_link_libraries(test_imu_correctness gtboxcalibration)

    catkin_add_gtest(test_offline_correctness tests/test_offline_correctness.cpp
            ros_camera_camera_program.cpp ros_camera_camera_offline_program.cpp
            ros_camera_camera_parser.cpp ros_utils.cpp)
    target_link_libraries(test_offline_correctness gtboxcalibration ${catkin_LIBRARIES})
    target_include_directories(test_offline_correctness PRIVATE ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})
    add_dependencies(test_offline_correctness ${catkin_EXPORTED_TARGETS})

    # Regression test — requires bag files on the external drive.
    # Run with: catkin_make run_tests
    catkin_add_gtest(test_calibration_regression
            tests/test_calibration_regression.cpp
            ros_camera_camera_program.cpp
            ros_camera_camera_offline_program.cpp
            ros_camera_camera_parser.cpp
            ros_utils.cpp
    )
    target_link_libraries(test_calibration_regression gtboxcalibration ${catkin_LIBRARIES})
    target_include_directories(test_calibration_regression PRIVATE
            ${catkin_INCLUDE_DIRS}
            ${CMAKE_CURRENT_SOURCE_DIR}
    )
endif ()

# Generated messages must exist before compiling the ROS adapters.
foreach(target camera_camera_online_calibration camera_camera_offline_calibration
        camera_prism_offline_calibration camera_imu_offline_calibration
        camera_imu_preprocess camera_imu_detection_benchmark)
    add_dependencies(${target} ${catkin_EXPORTED_TARGETS})
endforeach()
