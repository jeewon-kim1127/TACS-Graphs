#!/bin/bash

stop_existing_node() {
  kill $(ps -e | grep -vE "^ *$$|ps|grep|bash|dockerd|docker-containerd|docker" | awk '{print $1}')
  echo "stop_existing_node"
}
# Function to start sparse_graph
start_sparse_graph() {
  echo "Starting sparse_graph..."
  source /opt/ros/noetic/setup.bash
  source ~/workspaces/s_graphs_ros1_ws/devel/setup.bash
  roslaunch voxblox_skeleton skeletonize_map_realtime.launch >/dev/null 2>&1 &
  SPARSE_GRAPH_PID=$!
}

# Function to start ros1_bridge
start_ros1_bridge() {
  sleep 1
  echo "Starting ros1_bridge..."
  source /opt/ros/noetic/setup.bash
  source ~/workspaces/s_graphs_ros1_ws/devel/setup.bash
  source /opt/ros/foxy/setup.bash
  source ~/workspaces/s-graph/install/setup.bash
  ros2 run ros1_bridge dynamic_bridge >/dev/null 2>&1 &
  ROS1_BRIDGE_PID=$!
}

# Trap the SIGINT signal (Ctrl+C) and stop all processes
trap stop_existing_node SIGINT

sleep 2
# Start all processes
start_sparse_graph
start_ros1_bridge

# Wait for all background processes to finish
wait