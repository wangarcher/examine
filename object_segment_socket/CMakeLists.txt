cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(object_segment)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (object_segment object_segment.cpp)
target_link_libraries (object_segment ${PCL_LIBRARIES})
