cmake_minimum_required(VERSION 2.8)
project(mat_sophus)

# 为使用 sophus，需要使用find_package命令找到它
find_package(Sophus REQUIRED)

# Eigen
include_directories("/usr/include/eigen3")
add_executable(mat_sophus mat_sophus.cpp)
target_link_libraries(mat_sophus Sophus::Sophus)


