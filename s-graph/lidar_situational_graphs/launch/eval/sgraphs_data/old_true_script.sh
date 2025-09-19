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

# Stop all background processes
stop_existing_node() {
  kill $(ps -e | grep -vE "^ *$$|ps|grep|bash|dockerd|docker-containerd|docker" | awk '{print $1}')
  echo "stop_existing_node"
}
# Trap the SIGINT signal (Ctrl+C) and stop all processes
trap stop_existing_node SIGINT

stop_existing_node
# Start all processes
bash /root/workspaces/s-graph/lidar_situational_graphs/launch/eval/sgraphs_data/old_ros1_script.sh &
ROS1_PID=$!

ros2_script="/root/workspaces/s-graph/lidar_situational_graphs/launch/eval/sgraphs_data/old_true_ros2_script.sh"
/bin/bash "$ros2_script" "$output_file" "$rosbag_file" "$prefiltering_dist" "$distance_th" &
ROS2_PID=$!

# Wait for all background jobs to finish
wait $ROS2_PID
stop_existing_node