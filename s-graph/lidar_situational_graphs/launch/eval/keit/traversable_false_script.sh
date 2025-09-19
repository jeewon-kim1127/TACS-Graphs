#!/bin/bash
output_file=$1
rosbag_file=$2
prefiltering_dist=$3

# Check if the output_file is provided
if [ -z "$output_file" ]; then
  echo "No output_file parameter provided. Exiting."
  exit 1
fi

# Function to start the TRIP launch
start_trip() {
  echo "Starting TRIP..."
  source /opt/ros/foxy/setup.bash
  source /root/workspaces/trip_ws/install/setup.bash
  ros2 launch trip run_handheld_TRIP.py >/dev/null 2>&1 &
  TRIP_PID=$!
}

# Function to start the TRAVEL launch
start_travel() {
  echo "Starting TRAVEL..."
  source /opt/ros/foxy/setup.bash
  source /root/workspaces/trip_ws/install/setup.bash
  ros2 launch travel room_btms_trip_run_ros2.launch.py >/dev/null 2>&1 &
  TRAVEL_PID=$!
}

# Function to start the S-Graphs launch
start_s_graphs() {
  echo "Starting S-Graphs..."
  source /opt/ros/foxy/setup.bash
  source ~/workspaces/s-graph/install/setup.bash
  ros2 launch lidar_situational_graphs tiers7_launch.py room_segmentation:=traversable \
            compute_odom:=false output_file:=$output_file distance_far_thresh:=$prefiltering_dist  \
            lidar_topic:=/dreamstep_cloud_body odom_topic:=/dreamstep_odom imu_topic:=/go1_mti30/imu/data \
            base_frame:=base_imu_link base_link_frame:=base_imu_link &
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

# Function to play the Keit bag file
start_keitbag_play() {
  echo "Starting Keit bag playback..."
  sleep 1
  source /opt/ros/foxy/setup.bash
  source ~/workspaces/s-graph/install/setup.bash
  ros2 bag play $rosbag_file >/dev/null 2>&1 &
  ROSBAG_PID=$!
}

# Stop all background processes
stop_existing_node() {
  kill $(ps -e | grep -vE "^ *$$|ps|grep|bash|dockerd|docker-containerd|docker" | awk '{print $1}')
  echo "stop_existing_node"
}
# Trap the SIGINT signal (Ctrl+C) and stop all processes
trap stop_existing_node SIGINT

stop_existing_node

# Start all processes
start_trip
start_travel
start_s_graphs
# start_rviz
start_keitbag_play

# Wait for all background jobs to finish
wait $ROSBAG_PID

stop_existing_node