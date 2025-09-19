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
iterations=15

# Define the current script directory
current_launch_file_directory=$(dirname "$(readlink -f "$0")")

# Define the algorithms and datasets
algorithms=('traversable', 'old', 'new' )
datasets=('tiers7','tiers_1F','tiers_3F','tiers_6F' ) #'keit' 'tiers' 
compute_odom=('true') #'false'

# Iterate over each dataset, algorithm, and compute_odom value
for dataset in "${datasets[@]}"; do
  output_directory="${base_output_directory}/${dataset}"
  
  if [ ! -d "$output_directory" ]; then
    mkdir -p "${output_directory}"
  fi
  for algo in "${algorithms[@]}"; do
    if [ ! -d "${output_directory}/${algo}" ]; then
      mkdir -p "${output_directory}/${algo}"
    fi
    
    if [ "$algo" == "old" ]; then
      timers=('3.0' '5.0')
    elif [ "$algo" == "new" ]; then
      timers=('new')
    elif [ "$algo" == "traversable" ]; then
      timers=('traversable')
    fi

    for timer in ${timers[@]}; do
      if [ ! -d "${output_directory}/${algo}/timer_${timer}" ]; then
        mkdir -p "${output_directory}/${algo}/timer_${timer}"
      fi

      for compute_odom in "${compute_odom[@]}"; do
        script="${current_launch_file_directory}/${dataset}/${algo}_${compute_odom}_script.sh"
        if [ ! -f "$script" ]; then
          echo "Script $script not found, skipping."
          continue
        fi
        if [ ! -d "${output_directory}/${algo}/timer_${timer}/${compute_odom}" ]; then
          mkdir -p "${output_directory}/${algo}/timer_${timer}/${compute_odom}"
        fi
        for ((i=1; i<1+iterations; i++)); do
          output_file="${output_directory}/${algo}/timer_${timer}/${compute_odom}/iteration_${i}"
          
          # Execute the script with output file as an argument
          echo "Running ${script} with output file ${output_file} ${timer}"
          /bin/bash "$script" "$output_file" $timer &
          script_pid=$!
          # Wait for the background process to complete
          wait $script_pid
          # Check if the previous command was successful
          if [ $? -ne 0 ]; then
            echo "Error occurred during the execution of ${script}. Exiting."
            exit 1
          fi
        # Ensure the specific script's processes are terminated
        pkill -f ros2
        pkill -f ros
        sleep 2  # Add a small delay to ensure all processes are killed
        done
      done
    done
  done
done

echo "All processes completed successfully."
