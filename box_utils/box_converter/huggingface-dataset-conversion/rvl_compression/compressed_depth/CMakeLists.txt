cmake_minimum_required(VERSION 3.18)
project(compressed_depth LANGUAGES CXX)



# get nanobind git clone https://github.com/wjakob/nanobind.git --recursive
#     adds the python binding to decompress RVL compressed depth images from rosbag to PNGs

# Instructions:

#     git submodule update --init --recursive we added a new submodule nanobind , it should be located in: "grand_tour_box/box_utils/box_binding/compressed_depth/external"
#     cd grand_tour_box/box_utils/box_binding/compressed_depth
#     pip install -e . --no-build-isolation --verbose

# Go to grand_tour_box/box_utils/box_auto/python/box_auto/scripts/camera

# Set MISSION_DATA , WS variables, source the workspace for box_auto
# run the script

# The hacks that might be needed:

#     If running within docker D_PRELOAD=/usr/lib/x86_64-linux-gnu/libffi.so.7 python dummy.py

#     pip3 install nanobind scikit-build-core



find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
find_package(nanobind CONFIG REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

nanobind_add_module(_compressed_depth
    src/binding.cpp
    src/depth_rvl.cpp)

target_include_directories(_compressed_depth PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS})

target_link_libraries(_compressed_depth PRIVATE ${OpenCV_LIBS})

# put the .so inside the Python package directory
install(TARGETS _compressed_depth LIBRARY DESTINATION compressed_depth)
