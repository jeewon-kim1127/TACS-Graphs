cmake_minimum_required(VERSION 3.5)
project(tiers)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

add_executable(lidar_calibrate src/lidar_calibrate.cpp)
ament_target_dependencies(lidar_calibrate rclcpp tf2_ros sensor_msgs pcl_conversions pcl_ros)

add_executable(lidar_calibrate_2laps src/lidar_calibrate_2laps.cpp)
ament_target_dependencies(lidar_calibrate_2laps rclcpp tf2_ros sensor_msgs pcl_conversions pcl_ros)

install(TARGETS
  lidar_calibrate lidar_calibrate_2laps
  DESTINATION lib/${PROJECT_NAME})

ament_package()