cmake_minimum_required(VERSION 2.8)
project(CameraShow)
 
if(MSVC)
  option(OpenCV_STATIC "Use static OpenCV libraries" OFF)
endif()
 
find_package(OpenCV 3.2.0 REQUIRED)
 
include_directories(. ${OpenCV_INCLUDE_DIRS})
 
add_executable(CameraShow
               main.cpp)
target_link_libraries(CameraShow ${OpenCV_LIBS})
