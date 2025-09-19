#!/bin/bash
output_file=$1
rosbag_file=$2
prefiltering_dist=$3
distance_thresh=$4

if [ -z "$output_file" ]; then
  echo "No output_file parameter provided. Exiting."
  exit 1
fi


start_base_tf() {
  echo "Starting base_tf..."
  source /opt/ros/foxy/setup.bash
  source ~/workspaces/tiers_ws/install/setup.bash
  ros2 run tiers lidar_calibrate --ros-args -p use_sim_time:=true &
  BASE_TF_PID=$!
}

# Function to start rosbag playback
start_rosbag() {
  sleep 2
  echo "Starting $rosbag_file rosbag playback..."
  source /opt/ros/foxy/setup.bash
  source ~/workspaces/s-graph/install/setup.bash
  ros2 bag play $rosbag_file >/dev/null 2>&1 &
  ROSBAG_PID=$!
}

# Function to start s_graphs
start_s_graphs() {
  sleep 1
  echo "Starting s_graphs..."
  source /opt/ros/foxy/setup.bash
  source ~/workspaces/s-graph/install/setup.bash
  ros2 launch lidar_situational_graphs scan_matching_odometry_launch.py compute_odom:=true \
          output_file:=$output_file distance_far_thresh:=$prefiltering_dist distance_thresh:=$distance_thresh \
          lidar_topic:=/a_os0 imu_topic:=/os0_cloud_node/imu base_frame:=base_link &
  S_GRAPHS_PID=$!
}
# Function to start RViz
start_rviz() {
  echo "Starting RViz..."
  source /opt/ros/foxy/setup.bash
  source ~/workspaces/s-graph/install/setup.bash
  rviz2 -d ~/workspaces/s-graph/lidar_situational_graphs/rviz/s_graphs_ros2.rviz &
}


# Stop all background processes
stop_existing_node() {
  kill $(ps -e | grep -vE "^ *$$|ps|grep|bash|dockerd|docker-containerd|docker" | awk '{print $1}')
  echo "stop_existing_node"
  rm -rf ~/.ros/log/*
}
# Trap the SIGINT signal (Ctrl+C) and stop all processes
trap stop_existing_node SIGINT
stop_existing_node

start_base_tf
start_s_graphs
# start_rviz

start_rosbag

wait $ROSBAG_PID 
sleep 2

stop_existing_node
