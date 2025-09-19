from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from ruamel.yaml import YAML
import os

def generate_launch_description():
    yaml = YAML()
    package_share_dir = get_package_share_directory('vins')
    config_file = os.path.join(package_share_dir, '../../../..', 'src/mgs04/config/omo.yaml')
      
    with open(config_file) as f:
        config = yaml.load(f)
        
    params = {'config_file': config_file}
    robot_name = config['robot_name']
    imu_topic = config['imu_topic']
    img_topic = config['image0_topic']
    body_pose_topic = config['body_pose_topic']
    camera_pose_topic = config['camera_pose_topic']
    pointcloud_topic = config['pointcloud_topic']
    grid_map_topic = config['grid_map_topic']
    goal_pose_topic = config['goal_pose_topic']
    gpp_grid_topic = config['gpp_grid_topic']
    gpp_marker_topic = config['gpp_marker_topic']
    gpp_waypoint_topic = config['gpp_waypoint_topic'] 
    gpp_next_topic = config['gpp_next_topic']
    gpp_splined_topic = config['gpp_splined_topic']
    
    print(robot_name)
    print(gpp_grid_topic)
    
    return LaunchDescription([

        Node(
            package='gpp',
            executable='gpp_node',
            name='gpp_node',
            remappings=[
                (body_pose_topic, robot_name + body_pose_topic),
                (grid_map_topic, robot_name + grid_map_topic),
                (goal_pose_topic, robot_name + goal_pose_topic),
                (gpp_grid_topic, robot_name + gpp_grid_topic),
                (gpp_marker_topic, robot_name + gpp_marker_topic), 
                (gpp_waypoint_topic, robot_name + gpp_waypoint_topic),               
                (gpp_next_topic, robot_name + gpp_next_topic),
                (gpp_splined_topic, robot_name + gpp_splined_topic),
            ],
            output='screen'),
    ])
