
cmake_minimum_required(VERSION 3.1)

project(camera_calibration)
set(CMAKE_CXX_STANDARD 11)
add_executable(camera_calibration main.cpp)

find_package(OpenCV 3.2.0 REQUIRED)
target_link_libraries(camera_calibration ${OpenCV_LIBS})
