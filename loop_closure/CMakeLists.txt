cmake_minimum_required(VERSION 2.8)
project(loop_closure)

set(CMAKE_BUILD_TYPE "Debug")

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


include_directories(./)
add_executable(loop_closure loop_closure.cpp)
target_link_libraries (loop_closure ${PCL_LIBRARIES} ${Boost_LIBRARIES})


