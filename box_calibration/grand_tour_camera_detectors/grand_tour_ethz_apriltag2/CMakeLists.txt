cmake_minimum_required(VERSION 3.0.2)
project(grand_tour_ethz_apriltag2)

add_definitions(-fPIC -O3)

option(BUILD_EXAMPLES "Description of the option" FALSE)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

#library
file(GLOB SOURCE_FILES "src/*.cc")
add_library(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME}
        PUBLIC
        include ${EIGEN3_INCLUDE_DIRS})

#demo
if (BUILD_EXAMPLES)
    if (NOT APPLE)
        add_executable(apriltags_demo src/example/apriltags_demo.cpp src/example/Serial.cpp)
        target_link_libraries(apriltags_demo ${PROJECT_NAME} v4l2)
    endif ()
endif ()
