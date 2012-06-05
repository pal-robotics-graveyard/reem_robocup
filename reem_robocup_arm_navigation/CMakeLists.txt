cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rosbuild_add_executable(refresh_collision_map src/refresh_collision_map.cpp)
rosbuild_add_executable(dummy_cloud_publisher src/dummy_cloud_publisher.cpp)