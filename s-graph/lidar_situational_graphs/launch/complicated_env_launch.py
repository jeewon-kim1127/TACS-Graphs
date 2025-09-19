import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition, LaunchConfigurationEquals

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lidar_topic",
                default_value="platform/velodyne_points",
                description="Name of the lidar topic to sub",
            ),
            DeclareLaunchArgument(
                "trip_point_topic",
                default_value="/trip/trip_updated/alocal_dense_cloud",
                description="Name of the lidar topic to sub",
            ),

            DeclareLaunchArgument(
                "odom_topic",
                default_value="/platform/odometry", #/dreamstep_odom",
                description="Name of the odom topic to sub",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/go1_mti30/imu/data",
                description="Name of the imu topic to sub",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="body",
                description="Name of the base frame",
            ),
            DeclareLaunchArgument(
                "odom_frame",
                default_value="odom",
                description="Name of the base frame",
            ),
            DeclareLaunchArgument(
                "map_frame",
                default_value="map",
                description="Name of the base frame",
            ),
            DeclareLaunchArgument(
                "compute_odom",
                default_value="false",
                description="Flag to compute the odometry",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Namespace for the robot",
            ),
            DeclareLaunchArgument(
                "room_segmentation",
                default_value="traversable",
                description="Algorithm used for room segmentation",
            ),
            DeclareLaunchArgument(
                "debug_mode",
                default_value="false",
                description="Run s_graphs node in debugging mode",
            ),
            DeclareLaunchArgument(
                "output_file",
                default_value="/root/workspaces/data",
                description="output file directory",
            ),
            DeclareLaunchArgument(
                "main_timer_interval",
                default_value="1.0",
                description="main timer interval",
            ),
            DeclareLaunchArgument(
                "distance_far_thresh",
                default_value="50.0",
                description="distance far threshold for scan matching odometry",
            ),
            OpaqueFunction(function=launch_sgraphs),
        ]
    )

def launch_reasoning():
    reasoning_dir = get_package_share_directory('situational_graphs_reasoning')
    reasoning_launch_file = os.path.join(reasoning_dir, "launch", "situational_graphs_reasoning.launch.py")
    reasoning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(reasoning_launch_file))

    return reasoning_launch


def launch_sgraphs(context, *args, **kwargs):
    pkg_dir = get_package_share_directory("lidar_situational_graphs")
    prefiltering_param_file = os.path.join(pkg_dir, "config", "prefiltering.yaml")
    scan_matching_param_file = os.path.join(pkg_dir, "config", "scan_matching.yaml")
    s_graphs_param_file = os.path.join(pkg_dir, "config", "s_graphs.yaml")

    lidar_topic_arg = LaunchConfiguration("lidar_topic").perform(context)
    trip_point_topic_arg = LaunchConfiguration("trip_point_topic").perform(context)
    odom_topic_arg = LaunchConfiguration("odom_topic").perform(context)
    imu_topic_arg = LaunchConfiguration("imu_topic").perform(context)

    base_frame_arg = LaunchConfiguration("base_frame").perform(context)
    odom_frame_arg = LaunchConfiguration("odom_frame").perform(context)
    map_frame_arg = LaunchConfiguration("map_frame").perform(context)

    compute_odom_arg = LaunchConfiguration("compute_odom").perform(context)
    namespace_arg = LaunchConfiguration("namespace").perform(context)
    room_segmentation_arg =  LaunchConfiguration("room_segmentation").perform(context)
    debug_mode_arg =  LaunchConfiguration("debug_mode").perform(context)

    output_file_arg =  LaunchConfiguration("output_file").perform(context)
    main_timer_interval_arg =  LaunchConfiguration("main_timer_interval")
    distance_far_thresh_arg =  LaunchConfiguration("distance_far_thresh")
    
    ns_prefix = str(namespace_arg) + "/" if namespace_arg else ""
    if str(ns_prefix).startswith("/"):
        ns_prefix = ns_prefix[1:]

    base_link_frame = base_frame_arg
    odom_frame = odom_frame_arg
    map_frame = map_frame_arg


    prefiltering_cmd = Node(
        package="lidar_situational_graphs",
        executable="s_graphs_prefiltering_node",
        namespace=namespace_arg,
        parameters=[{prefiltering_param_file}, 
                        {"base_link_frame": base_link_frame,
                        "distance_far_thresh": distance_far_thresh_arg,
                        "downsample_resolution": 0.1}],
        output="screen",
        remappings=[
            ("velodyne_points", lidar_topic_arg),
            # ("trip_points", trip_point_topic_arg),
            ("imu/data", imu_topic_arg),
        ],
    )

    scan_matching_cmd = Node(
        package="lidar_situational_graphs",
        executable="s_graphs_scan_matching_odometry_node",
        namespace=namespace_arg,
        parameters=[{scan_matching_param_file}],
        remappings=[("odom", odom_topic_arg)],
        output="screen",
        condition=IfCondition(compute_odom_arg),
    )

    if room_segmentation_arg == "traversable":
        subroom_segmentation_cmd = Node(
            package="lidar_situational_graphs",
            executable="subroom_segmentation_node",
            namespace=namespace_arg,
            parameters=[{"vertex_neigh_thres": 10, 
                        "cluster_resolution": 0.05, 
                        "output_file": output_file_arg}],
            output="screen",
        )

    floor_plan_cmd = Node(
        package="lidar_situational_graphs",
        executable="s_graphs_floor_plan_node",
        namespace=namespace_arg,
        parameters=[{"vertex_neigh_thres": 2}],
        output="screen",
    )

    s_graphs_cmd = Node(
        package="lidar_situational_graphs",
        executable="s_graphs_node",
        namespace=namespace_arg,
        parameters=[{s_graphs_param_file}, {
                    "odom_frame_id": odom_frame, "map_frame_id": map_frame, 
                    "distance_thresh": 3.0, "roompart_length": 3.0,
                    "output_file": output_file_arg, "main_timer_interval": main_timer_interval_arg, "read_ahead_queue_size": 100 }],
        output="screen",
        prefix=["gdbserver localhost:3000"] if debug_mode_arg == "true" else None,
        remappings=[
            ("velodyne_points", lidar_topic_arg),
            # ("filtered_points", "trip_filtered_points"),
            ("odom", odom_topic_arg)
        ],
    )

    map_keyframe_static_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_keyframe_static_transform",
        arguments=[
            "0.0",
            "0.0",
            "7.0",
            "0.0",
            "0.0",
            "0.0",
            ns_prefix + "map",
            ns_prefix + "keyframes_layer",
        ],
        output="screen",
    )

    keyframe_wall_static_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="keyframe_wall_static_transform",
        arguments=[
            "0.0",
            "0.0",
            "8.0",
            "0.0",
            "0.0",
            "0.0",
            ns_prefix + "keyframes_layer",
            ns_prefix + "walls_layer",
        ],
        output="screen",
    )

    wall_room_static_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="wall_room_static_transform",
        arguments=[
            "0.0",
            "0.0",
            "7.0",
            "0.0",
            "0.0",
            "0.0",
            ns_prefix + "walls_layer",
            ns_prefix + "rooms_layer",
        ],
        output="screen",
    )

    room_floor_static_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="room_floor_static_transform",
        arguments=[
            "0.0",
            "0.0",
            "5.0",
            "0.0",
            "0.0",
            "0.0",
            ns_prefix + "rooms_layer",
            ns_prefix + "floors_layer",
        ],
        output="screen",
    )
    return [
        prefiltering_cmd,
        scan_matching_cmd,
        subroom_segmentation_cmd,
        floor_plan_cmd,
        s_graphs_cmd,
        map_keyframe_static_transform,
        keyframe_wall_static_transform,
        wall_room_static_transform,
        room_floor_static_transform,
    ]