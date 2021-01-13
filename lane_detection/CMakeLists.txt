cmake_minimum_required(VERSION 2.8)
project(lane_detection)


set(CMAKE_BUILD_TYPE "Release")
# 添加c++ 11标准支持
set(CMAKE_CXX_FLAGS "-std=c++11 -O2")

# Eigen
include_directories("/usr/include/eigen3")

# 寻找OpenCV库
find_package(OpenCV 3.2.0 REQUIRED)
# 添加头文件
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(lane_detection lane_detection.cpp)
# 链接OpenCV库
target_link_libraries(lane_detection ${OpenCV_LIBS})


