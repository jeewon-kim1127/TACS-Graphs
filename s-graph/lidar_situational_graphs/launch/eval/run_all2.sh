#!/bin/bash

# Stop all background processes
stop_all() {
  echo "Stopping all processes..."
  kill -- -$$
}

# Trap the SIGINT signal (Ctrl+C) and stop all processes
trap stop_all SIGINT

# Define the base output directory and iterations
base_output_directory='/root/workspaces/data'
i_start=1
iterations=5

echo "base_output_directory $base_output_directory"

# Define the current script directory
current_launch_file_directory=$(dirname "$(readlink -f "$0")")

# Define the algorithms and datasets
algorithms=("gicp") #("old" "traversable" "old_ablation" "new")
datasets=( 'Indoor_6F') #'Indoor7' 'Indoor_1F' 'Indoor_6F' 'Indoor_3F' )
dists=('15.0' '50.0')
loop_dist_ths=('1.0')
#### loop dist th 5.0

# Iterate over each dataset, algorithm, 
for algo in "${algorithms[@]}"; do
  for dataset in "${datasets[@]}"; do
    dataset=$(echo "$dataset" | xargs)
    if [ ! -d "${base_output_directory}/${dataset}" ]; then
      mkdir -p "${base_output_directory}/${dataset}"
    fi
    if [ ! -d "${base_output_directory}/${dataset}/${algo}" ]; then
      mkdir -p "${base_output_directory}/${dataset}/${algo}"
    fi
    
    if [[ "$dataset" == "Indoor_3F" ]]; then
      script="${current_launch_file_directory}/tiers/${algo}_script_twice.sh"
    else
      script="${current_launch_file_directory}/tiers/${algo}_script_once.sh"
    fi
    if [ ! -f "$script" ]; then
        echo "Script $script not found, skipping."
        continue
    fi 

    if [[ "$dataset" == "Indoor_6F" ]] ; then
      rosbag="/media/URL_G1/Tiers/indoor/${dataset}/${dataset}.db3"
    else
      rosbag="/root/Downloads/${dataset}/${dataset}.db3"
    fi

    for prefiltering_dist in ${dists[@]}; do
      for loop_dist_th in ${loop_dist_ths[@]}; do
        for ((i=i_start; i<i_start+iterations; i++)); do
          output_file="${base_output_directory}/${dataset}/${algo}/${prefiltering_dist}_${loop_dist_th}_it${i}"
          
          echo "Running ${script} with output file ${output_file} prefilter: ${prefiltering_dist} loop: ${loop_dist_th}"
          /bin/bash "$script" "$output_file" "$rosbag" "$prefiltering_dist" "$loop_dist_th" &
          script_pid=$!
          wait $script_pid
          if [ $? -ne 0 ]; then
            echo "Error occurred during the execution of ${script}. Exiting."
            exit 1
          fi
          pkill -f ros2
          pkill -f ros
          sleep 2  # Add a small delay to ensure all processes are killed
        done
      done
    done
  done
done

echo "All processes completed successfully."
