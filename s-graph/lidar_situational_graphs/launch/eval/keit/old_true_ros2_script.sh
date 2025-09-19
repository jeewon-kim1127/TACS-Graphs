#!/bin/bash
output_file=$1
rosbag_file=$2
prefiltering_dist=$3
distance_th=$4

# Check if the output_file is provided
if [ -z "$output_file" ]; then
  echo "No output_file parameter provided. Exiting."
  exit 1
fi

# Function to start rosbag playback
start_rosbag() {
  sleep 2
  echo "Starting rosbag playback..."
  source /opt/ros/foxy/setup.bash
  source ~/workspaces/s-graph/install/setup.bash
  ros2 bag play $rosbag_file >/dev/null 2>&1 &
  ROSBAG_PID=$!
}

# Function to start s_graphs
start_s_graphs() {
  sleep 2
  echo "Starting s_graphs..."
  source /opt/ros/foxy/setup.bash
  source ~/workspaces/s-graph/install/setup.bash
  ros2 launch lidar_situational_graphs s_graphs_launch.py room_segmentation:=old \
      compute_odom:=true output_file:=$output_file distance_far_thresh:=$prefiltering_dist distance_thresh:=$distance_th \
      lidar_topic:=/dreamstep_cloud_body imu_topic:=/go1_mti30/imu/data base_frame:=base_imu_link &
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
stop_existing_node() {
  kill $(ps -e | grep -vE "^ *$$|ps|grep|bash|dockerd|docker-containerd|docker" | awk '{print $1}')
  echo "stop_existing_node"
}
# Trap the SIGINT signal (Ctrl+C) and stop all processes
trap stop_existing_node SIGINT

sleep 2
# Start all processes
start_rosbag
start_s_graphs
# start_rviz

# Wait for all background processes to finish
wait $ROSBAG_PID
stop_existing_node