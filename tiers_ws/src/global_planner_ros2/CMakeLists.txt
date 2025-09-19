cmake_minimum_required(VERSION 3.5)
project(gpp)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wextra -Wpedantic)

set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rcpputils REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(image_geometry REQUIRED)
#find_package(saslam_msg REQUIRED)

# find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)

find_package(mgs04_base_msgs REQUIRED)

find_package(PCL REQUIRED )
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${CERES_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


add_library(gpp_lib
    src/gpp/pathplan.h
    src/gpp/pathplan.cpp
    src/gpp/planner/aStarLibrary.cpp
    src/gpp/planner/aStarLibrary.h
    src/gpp/planner/Astarpp.cpp
    src/gpp/planner/Astarpp.h
    src/gpp/planner/gridlinetraversal.h
    src/gpp/planner/ShortestPP.cpp
    src/gpp/planner/ShortestPP.h
    src/gpp/planner/Struct.h
)
target_link_libraries(gpp_lib  ${CERES_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
ament_target_dependencies(gpp_lib mgs04_base_msgs rclcpp rclpy rcpputils  tf2 tf2_ros visualization_msgs geometry_msgs nav_msgs std_msgs sensor_msgs gazebo_msgs  pcl_conversions laser_geometry image_geometry)

add_executable(gpp_node src/gpp_node.cpp)
ament_target_dependencies(gpp_node mgs04_base_msgs rclcpp rclpy rcpputils  tf2 tf2_ros visualization_msgs geometry_msgs nav_msgs std_msgs sensor_msgs gazebo_msgs  pcl_conversions laser_geometry image_geometry)
target_link_libraries(gpp_node gpp_lib) 


# added from DEBUG

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install nodes
install(
  TARGETS gpp_node
  DESTINATION lib/${PROJECT_NAME}
)



# Install binaries
#install(TARGETS pathplan_node kitti_odom_test kitti_gps_test vins_lib
#    ARCHIVE DESTINATION lib
#    LIBRARY DESTINATION lib
#    RUNTIME DESTINATION bin
#)

ament_package()
