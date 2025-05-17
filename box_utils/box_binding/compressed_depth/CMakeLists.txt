cmake_minimum_required(VERSION 3.18)
project(compressed_depth LANGUAGES CXX)

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
