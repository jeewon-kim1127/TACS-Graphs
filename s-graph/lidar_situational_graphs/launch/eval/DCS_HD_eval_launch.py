import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import os

def generate_launch_description():
    current_launch_file_directory = os.path.dirname(os.path.abspath(__file__))
    # Parameters for algorithms and datasets
    algorithms = ['old', 'traversable', 'new']
    datasets = [#'tiers', 
                'keit']
    base_output_directory = '/root/data'
    iterations = 10

    # Declare launch arguments for the algorithm and dataset
    algo_arg = DeclareLaunchArgument(
        'algorithm', default_value='old',
        description='Algorithm to use: old, traversable, new'
    )
    data_arg = DeclareLaunchArgument(
        'dataset', default_value='keit',
        description='Dataset to use: tiers, keit'
    )
    # Paths to mprocs configurations
    launch_file = {
        'old': 's_graphs_launch_old.py',
        'traversable': 's_graphs_launch_traversable.py',
        'new': 's_graphs_launch_new.py',
    }

    # Create a list of ExecuteProcess actions to run the algorithm with each combination
    processes = []

    for dataset in datasets:
        output_directory = os.path.join(base_output_directory, f"{dataset}")
        for algo in algorithms:
            for compute_odom in ['true','false']:
                script = os.path.join(current_launch_file_directory, f'./{dataset}_{algo}_{compute_odom}_script.sh')
                for i in range(iterations):
                    output_file = os.path.join(output_directory, f'{algo}/iteration_{i}')
                    process = ExecuteProcess(
                        cmd=['/bin/bash', script],
                        output='screen',
                        name=f'run_{algo}_{dataset}_computeodom_{compute_odom}_iteration_{i}',
                        parameters=[
                            {'output_file': output_file}
                        ]
                    )
                    processes.append(process)

    return LaunchDescription(processes)
