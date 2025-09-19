#!/bin/bash

# Capture the output_file parameter passed to this script
output_file=$1
rosbag_file=$2
prefiltering_dist=$3

# Check if the output_file is provided
if [ -z "$output_file" ]; then
  echo "No output_file parameter provided. Exiting."
  exit 1
fi

stop_existing_node() {
  kill $(ps -e | grep -vE "^ *$$|ps|grep|bash|dockerd|docker-containerd|docker" | awk '{print $1}')
  echo "stop_existing_node"
}

start_base_tf() {
  echo "Starting base_tf..."
  source /opt/ros/foxy/setup.bash
  source ~/workspaces/tiers_ws/install/setup.bash
  ros2 run tiers lidar_calibrate &
  BASE_TF_PID=$!
}

# Function to start rosbag playback
start_rosbag() {
  sleep 1
  echo "Starting Indoor10 rosbag playback..."
  source /opt/ros/foxy/setup.bash
  source ~/workspaces/s-graph/install/setup.bash
  ros2 bag play $rosbag_file >/dev/null 2>&1 &
  ROSBAG_PID=$!
}

# Function to start s_graphs
start_s_graphs() {
  echo "Starting s_graphs..."
  source /opt/ros/foxy/setup.bash
  source ~/workspaces/s-graph/install/setup.bash
  ros2 launch lidar_situational_graphs ablation_s_graphs_launch.py room_segmentation:=old compute_odom:=true \
        output_file:=$output_file distance_far_thresh:=$prefiltering_dist \
        lidar_topic:=/a_os0 imu_topic:=/os0_cloud_node/imu base_frame:=base_link &
  S_GRAPHS_PID=$!
}

# Function to start RViz
start_rviz() {
  echo "Starting RViz..."
  source /opt/ros/foxy/setup.bash
  source ~/workspaces/s-graph/install/setup.bash
  rviz2 -d ~/workspaces/s-graph/lidar_situational_graphs/rviz/s_graphs_ros2.rviz &
  RVIZ_PID=$!
}

# Stop all background processes
stop_all() {
  echo "Stopping all processes..."
  kill $S_GRAPHS_PID $BASE_TF_PID $ROSBAG_PID $RVIZ_PID
}
# Trap the SIGINT signal (Ctrl+C) and stop all processes
trap stop_all SIGINT

sleep 5

start_s_graphs
start_base_tf
start_rviz

# i=0
# Loop until the counter reaches 10
# while [ $i -lt 2 ]; do
start_rosbag

  # Wait for all background processes to finish
wait $ROSBAG_PID
  # i=$((i + 1))
  # sleep 2

  # if [ $i -ge 2 ]; then
echo "Maximum iterations reached. Terminating."
sleep 5
#     break
#   fi
# done

stop_existing_node