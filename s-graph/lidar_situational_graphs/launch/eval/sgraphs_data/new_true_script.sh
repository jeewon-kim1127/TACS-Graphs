#!/bin/bash
output_file=$1
rosbag_file=$2
prefiltering_dist=$3

# Check if the output_file is provided
if [ -z "$output_file" ]; then
  echo "No output_file parameter provided. Exiting."
  exit 1
fi

# Function to start rosbag playback
start_rosbag() {
  echo "Starting rosbag playback..."
  sleep 1
  source /opt/ros/foxy/setup.bash
  source ~/workspaces/s-graph/install/setup.bash
  ros2 bag play $rosbag_file >/dev/null 2>&1 &
  ROSBAG_PID=$!
}

start_s_graphs_simulated() {
  echo "Starting s_graphs..."
  source /opt/ros/foxy/setup.bash
  source ~/workspaces/s-graph/install/setup.bash
  ros2 launch lidar_situational_graphs s_graphs_launch.py room_segmentation:=new \
        compute_odom:=true base_link_frame:=base_footprint \
        output_file:=$output_file distance_far_thresh:=$prefiltering_dist &
  S_GRAPHS_PID=$!
}
# Function to start s_graphs
start_s_graphs_real() {
  echo "Starting s_graphs..."
  source /opt/ros/foxy/setup.bash
  source ~/workspaces/s-graph/install/setup.bash
  ros2 launch lidar_situational_graphs s_graphs_launch.py room_segmentation:=new \
      compute_odom:=false base_link_frame:=body \
      output_file:=$output_file distance_far_thresh:=$prefiltering_dist &
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


# Start all processes
start_s_graphs_simulated
#start_s_graphs_real
start_rosbag
# start_rviz

# Wait for all background jobs to finish
wait $ROSBAG_PID

echo "All processes have been stopped."
stop_existing_node