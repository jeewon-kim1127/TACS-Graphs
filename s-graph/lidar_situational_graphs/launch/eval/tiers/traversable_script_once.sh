#!/bin/bash
# Capture the output_file parameter passed to this script
output_file=$1
rosbag_file=$2
prefiltering_dist=$3

if [ -z "$output_file" ]; then
  echo "No output_file parameter provided. Exiting."
  exit 1
fi
if [ -z "$rosbag_file" ]; then
  echo "No rosbag_file parameter provided. Exiting."
  exit 1
fi

stop_existing_node() {
  kill $(ps -e | grep -vE "^ *$$|ps|grep|bash|dockerd|docker-containerd|docker" | awk '{print $1}')
  echo "stop_existing_node"
}

# Function to start the TRIP launch
start_trip() {
  echo "Starting TRIP..."
  source /opt/ros/foxy/setup.bash
  source /root/workspaces/trip_ws/install/setup.bash
  ros2 launch trip run_room_TRIP.py >/dev/null 2>&1 &
  TRIP_PID=$!
}

# Function to start the TRAVEL launch
start_travel() {
  echo "Starting TRAVEL..."
  source /opt/ros/foxy/setup.bash
  source /root/workspaces/trip_ws/install/setup.bash
  ros2 launch travel corridor_btms_trip_run_ros2.launch.py >/dev/null 2>&1 &
  TRAVEL_PID=$!
}

# Function to start the S-Graphs launch
start_s_graphs() {
  echo "Starting S-Graphs..."
  source /opt/ros/foxy/setup.bash
  source ~/workspaces/s-graph/install/setup.bash
  ros2 launch lidar_situational_graphs tiers7_launch.py room_segmentation:=traversable compute_odom:=true output_file:=$output_file distance_far_thresh:=$prefiltering_dist \
                            lidar_topic:=/a_os0 imu_topic:=/os0_cloud_node/imu base_frame:=base_link &
  # ros2 launch lidar_situational_graphs ts_graphs_launch.py room_segmentation:=traversable compute_odom:=true output_file:=$output_file lidar_topic:=/a_os0 trip_point_topic:=/trip/trip_updated/alocal_dense_cloud imu_topic:=/os0_cloud_node/imu base_frame:=base_link &

  S_GRAPHS_PID=$!
}

# Function to start RViz
start_rviz() {
  echo "Starting RViz..."
  source /opt/ros/foxy/setup.bash
  source ~/workspaces/s-graph/install/setup.bash
  rviz2 -d ~/workspaces/s-graph/lidar_situational_graphs/rviz/s_graphs_ros2.rviz &
  #RVIZ_PID=$!
}

start_base_tf() {
  echo "Starting base_tf..."
  source /opt/ros/foxy/setup.bash
  source /root/workspaces/tiers_ws/install/setup.bash
  ros2 run tiers lidar_calibrate --ros-args -p use_sim_time:=true &
  BASE_TF_PID=$!
}

# Function to play the Keit bag file
start_rosbag() {
  echo "Starting $rosbag_file playback..."
  source /opt/ros/foxy/setup.bash
  source ~/workspaces/s-graph/install/setup.bash
  ros2 bag play $rosbag_file >/dev/null 2>&1 &
  ROSBAG_PID=$!
}

# Trap the SIGINT signal (Ctrl+C) and stop all processes
trap stop_existing_node SIGINT

stop_existing_node

sleep 1
start_base_tf
# start_rviz

start_trip
start_travel
start_s_graphs

# i=0
# Loop until the counter reaches 10
# while [ $i -lt 2 ]; do
  # start_trip
  # start_travel
echo "Playing the rosbag: Attempt $i"
start_rosbag

# Wait for all background processes to finish
wait $ROSBAG_PID 
  # kill $TRIP_PID $TRAVEL_PID
  
#   i=$((i + 1))

#   if [ $i -ge 2 ]; then
sleep 5
#     break
#   fi
# done
# echo "Maximum iterations reached. Terminating."

stop_existing_node