cmake_minimum_required(VERSION 3.0.2)
project(box_calibration)
find_package(catkin REQUIRED COMPONENTS
        rospy
        std_msgs
        sensor_msgs
)
catkin_python_setup()

catkin_package(
        CATKIN_DEPENDS rospy std_msgs sensor_msgs
)

# Install Python scripts
install(PROGRAMS
        scripts/convert_ceres_camera_output_to_kalibr_style.py
        scripts/import_calibration.py
        scripts/populate_camera_calibrations.py
        scripts/process_all_calibrations.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
