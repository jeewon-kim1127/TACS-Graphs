#!/bin/bash
output_file=$1
rosbag_file=$2
prefiltering_dist=$3

stop_existing_node() {
  kill $(ps -e | grep -vE "^ *$$|ps|grep|bash|dockerd|docker-containerd|docker" | awk '{print $1}')
  echo "stop_existing_node"
}

stop_existing_node

# Check if the output_file is provided
if [ -z "$output_file" ]; then
  echo "No output_file parameter provided. Exiting."
  exit 1
else
  echo "Output file is set to: $output_file"
fi

# Start all processes
bash /root/workspaces/s-graph/lidar_situational_graphs/launch/eval/tiers/old_ros1_script.sh &
ROS1_PID=$!

sleep 5

ros2_script="/root/workspaces/s-graph/lidar_situational_graphs/launch/eval/tiers/old_ablation_ros2_script_twice.sh"
/bin/bash "$ros2_script" "$output_file" "$rosbag_file" "$prefiltering_dist" &
ROS2_PID=$!

# Stop all background processes
stop_all() {
  echo "Stopping all processes..."
  kill $ROS1_PID $ROS2_PID #-- -$$
}
# Trap the SIGINT signal (Ctrl+C) and stop all processes
trap stop_all SIGINT

# Wait for all background jobs to finish
wait $ROS2_PID

stop_all
stop_existing_node