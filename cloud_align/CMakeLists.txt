cmake_minimum_required(VERSION 2.6)
project(cloud_align LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.11.0 EXACT REQUIRED)
find_package(VTK 8.2.0 EXACT REQUIRED)

set  (CMAKE_BUILD_TYPE "Release")
set  (CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb -static-libgcc -static-libstdc++")
set  (CMAKE_CXX_FLAGS_RELEAS "$ENV{CXXFLAGS} -O3 -Wall -pthread -static-libgcc -static-libstdc++")

include_directories(${PCL_INCLUDE_DIRS} ${VTK_INCLUDE_DIR})
# eigen 
include_directories("/usr/include/eigen3/")

link_directories(${PCL_LIBRARY_DIRS} ${VTK_LIBRARY_DIR})
add_definitions(${PCL_DEFINITIONS})

add_executable(cloud_align cloud_align.cpp)

target_link_libraries (cloud_align ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${VTK_LIBRARIES})

