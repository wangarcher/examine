cmake_minimum_required(VERSION 2.6)
project(cloud_normal)


find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(cloud_normal cloud_normal.cpp)

target_link_libraries (cloud_normal ${PCL_LIBRARIES} ${Boost_LIBRARIES})

