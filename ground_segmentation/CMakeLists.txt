cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ground_segmentation)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (ground_segmentation ground_segmentation.cpp)
target_link_libraries (ground_segmentation ${PCL_LIBRARIES})
