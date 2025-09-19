cmake_minimum_required(VERSION 3.5)
project(situational_graphs_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PointClouds.msg"
  "msg/PlaneData.msg"
  "msg/PlanesData.msg"
  "msg/RoomData.msg"
  "msg/RoomsData.msg"
  "msg/CompleteRoomData.msg"
  "msg/CompleteRoomsData.msg"
  "msg/FloorCoeffs.msg"
  "msg/FloorData.msg"
  "msg/ScanMatchingStatus.msg"
  "msg/WallData.msg"
  "msg/WallsData.msg"
  "srv/DumpGraph.srv"
  "srv/SaveMap.srv"
  "srv/LoadGraph.srv"
  DEPENDENCIES builtin_interfaces rosidl_default_generators std_msgs geometry_msgs sensor_msgs visualization_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
