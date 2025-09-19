/*
Copyright (c) 2023, University of Luxembourg
All rights reserved.

Redistributions and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*/

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <Eigen/Dense>
#include <atomic>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <boost/thread.hpp>
#include <ctime>
#include <mutex>
#include <s_graphs/backend/floor_mapper.hpp>
#include <s_graphs/backend/gps_mapper.hpp>
#include <s_graphs/backend/graph_slam.hpp>
#include <s_graphs/backend/imu_mapper.hpp>
#include <s_graphs/backend/keyframe_mapper.hpp>
#include <s_graphs/backend/loop_mapper.hpp>
#include <s_graphs/backend/plane_mapper.hpp>
#include <s_graphs/backend/room_graph_generator.hpp>
#include <s_graphs/backend/infinite_room_graph_generator.hpp>
#include <s_graphs/backend/room_mapper_old.hpp>
#include <s_graphs/backend/wall_mapper.hpp>
#include <s_graphs/common/floors.hpp>
#include <s_graphs/common/graph_utils.hpp>
#include <s_graphs/common/infinite_rooms.hpp>
#include <s_graphs/common/information_matrix_calculator.hpp>
#include <s_graphs/common/keyframe.hpp>
#include <s_graphs/common/map_cloud_generator.hpp>
#include <s_graphs/common/nmea_sentence_parser.hpp>
#include <s_graphs/common/plane_utils.hpp>
#include <s_graphs/common/planes.hpp>
#include <s_graphs/common/room_utils.hpp>
#include <s_graphs/common/rooms.hpp>
#include <s_graphs/common/ros_time_hash.hpp>
#include <s_graphs/common/ros_utils.hpp>
#include <s_graphs/frontend/keyframe_updater.hpp>
#include <s_graphs/frontend/loop_detector.hpp>
#include <s_graphs/frontend/plane_analyzer.hpp>
#include <s_graphs/visualization/graph_publisher.hpp>
#include <s_graphs/visualization/graph_visualizer.hpp>
#include <unordered_map>

#include "geographic_msgs/msg/geo_point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nmea_msgs/msg/sentence.hpp"
#include "pcl_ros/transforms.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "situational_graphs_msgs/msg/floor_coeffs.hpp"
#include "situational_graphs_msgs/msg/plane_data.hpp"
#include "situational_graphs_msgs/msg/planes_data.hpp"
#include "situational_graphs_msgs/msg/point_clouds.hpp"
#include "situational_graphs_msgs/msg/room_data.hpp"
#include "situational_graphs_msgs/msg/rooms_data.hpp"
#include "situational_graphs_msgs/msg/complete_room_data.hpp"
#include "situational_graphs_msgs/msg/complete_rooms_data.hpp"
#include "situational_graphs_msgs/msg/wall_data.hpp"
#include "situational_graphs_msgs/msg/walls_data.hpp"
#include "situational_graphs_msgs/srv/dump_graph.hpp"
#include "situational_graphs_msgs/srv/load_graph.hpp"
#include "situational_graphs_msgs/srv/save_map.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer_interface.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <filesystem>

namespace s_graphs {

class SGraphsNode : public rclcpp::Node {
 public:
  typedef pcl::PointXYZI PointT;
  typedef pcl::PointXYZRGBNormal PointNormal;

  SGraphsNode() : Node("s_graphs_node") {
    // init ros parameters
    this->declare_ros_params();
    map_frame_id =
        this->get_parameter("map_frame_id").get_parameter_value().get<std::string>();
    odom_frame_id =
        this->get_parameter("odom_frame_id").get_parameter_value().get<std::string>();
    
    std::string ns = this->get_namespace();
    if (ns.length() > 1) {
      std::string ns_prefix = std::string(this->get_namespace()).substr(1);
      map_frame_id = ns_prefix + "/" + map_frame_id;
      odom_frame_id = ns_prefix + "/" + odom_frame_id;
    }

    map_cloud_resolution =
        this->get_parameter("map_cloud_resolution").get_parameter_value().get<double>();
    wait_trans_odom2map =
        this->get_parameter("wait_trans_odom2map").get_parameter_value().get<bool>();
    use_map2map_transform =
        this->get_parameter("use_map2map_transform").get_parameter_value().get<bool>();
    got_trans_odom2map = false;
    trans_odom2map.setIdentity();
    odom_path_vec.clear();

    max_keyframes_per_update = this->get_parameter("max_keyframes_per_update")
                                   .get_parameter_value()
                                   .get<int>();

    gps_time_offset =
        this->get_parameter("gps_time_offset").get_parameter_value().get<int>();
    imu_time_offset =
        this->get_parameter("imu_time_offset").get_parameter_value().get<int>();
    enable_imu_orientation =
        this->get_parameter("enable_imu_orientation").get_parameter_value().get<bool>();
    enable_imu_acceleration = this->get_parameter("enable_imu_acceleration")
                                  .get_parameter_value()
                                  .get<bool>();
    imu_orientation_edge_stddev = this->get_parameter("imu_orientation_edge_stddev")
                                      .get_parameter_value()
                                      .get<double>();
    imu_acceleration_edge_stddev = this->get_parameter("imu_acceleration_edge_stddev")
                                       .get_parameter_value()
                                       .get<double>();

    optimization_window_size = this->get_parameter("optimization_window_size")
                                   .get_parameter_value()
                                   .get<int>();

    keyframe_window_size =
        this->get_parameter("keyframe_window_size").get_parameter_value().get<int>();
    extract_planar_surfaces = this->get_parameter("extract_planar_surfaces")
                                  .get_parameter_value()
                                  .get<bool>();
    constant_covariance =
        this->get_parameter("constant_covariance").get_parameter_value().get<bool>();

    infinite_room_information = this->get_parameter("infinite_room_information")
                                    .get_parameter_value()
                                    .get<double>();
    room_information =
        this->get_parameter("room_information").get_parameter_value().get<double>();
    std::cout<<"infinite_room_information: "<<infinite_room_information<<" room_information "<<room_information<<std::endl;
    points_topic =
        this->get_parameter("points_topic").get_parameter_value().get<std::string>();

    save_info = this->get_parameter("save_info").get_parameter_value().get<bool>();
    save_timings = this->get_parameter("save_timings").get_parameter_value().get<bool>();

    output_file = this->get_parameter("output_file").get_parameter_value().get<std::string>();
    main_timer_interval = this->get_parameter("main_timer_interval").get_parameter_value().get<double>();
    
    use_old_ablation = this->get_parameter("use_old_ablation")
                                .get_parameter_value().get<bool>();

    // slam_filename = output_file+"_slam.csv";
    slam_loop_filename = output_file+"_slam_loop.csv";
    
    // tfs
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    odom2map_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    callback_group_subscriber =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_subscriber;

    // subscribers
    init_odom2map_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "odom2map/initial_pose",
        1,
        std::bind(
            &SGraphsNode::init_map2odom_pose_callback, this, std::placeholders::_1),
        sub_opt);
    while (wait_trans_odom2map && !got_trans_odom2map) {
      RCLCPP_INFO(this->get_logger(),
                  "Waiting for the Initial Transform between odom and map frame");
      rclcpp::spin_some(shared_from_this());
      usleep(1e6);
    }
    map_2map_transform_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "map2map/transform",
        1,
        std::bind(
            &SGraphsNode::map2map_transform_callback, this, std::placeholders::_1),
        sub_opt);

    odom_sub.subscribe(this, "odom");
    cloud_sub.subscribe(this, "filtered_points");
    sync.reset(new message_filters::Synchronizer<ApproxSyncPolicy>(
        ApproxSyncPolicy(32), odom_sub, cloud_sub));
    sync->registerCallback(&SGraphsNode::cloud_callback, this);

    raw_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom",
        1,
        std::bind(&SGraphsNode::raw_odom_callback, this, std::placeholders::_1),
        sub_opt);

    point_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "filtered_points",
        5,
        std::bind(&SGraphsNode::point_cloud_callback, this, std::placeholders::_1),
        sub_opt);

    callback_group_publisher =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto pub_opt = rclcpp::PublisherOptions();
    pub_opt.callback_group = callback_group_publisher;

    // publishers
    markers_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "s_graphs/markers", 1, pub_opt);
    odom2map_pub = this->create_publisher<geometry_msgs::msg::TransformStamped>(
        "s_graphs/odom2map", 16, pub_opt);
    odom_corrected_pub = this->create_publisher<nav_msgs::msg::Odometry>(
        "s_graphs/odom_corrected", 20, pub_opt);
    odom_pose_corrected_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "s_graphs/odom_pose_corrected", 1, pub_opt);
    odom_path_corrected_pub = this->create_publisher<nav_msgs::msg::Path>(
        "s_graphs/odom_path_corrected", 1, pub_opt);

    read_until_pub = this->create_publisher<std_msgs::msg::Header>(
        "s_graphs/read_until", 32, pub_opt);
    graph_pub = this->create_publisher<situational_graphs_reasoning_msgs::msg::Graph>(
        "s_graphs/graph_structure", 32, pub_opt);

    loop_found = false;
    duplicate_planes_found = false;
    global_optimization = false;
    graph_updated = false;
    prev_edge_count = curr_edge_count = 0;

    double graph_update_interval = this->get_parameter("graph_update_interval")
                                       .get_parameter_value()
                                       .get<double>();
    double keyframe_timer_update_interval =
        this->get_parameter("keyframe_timer_update_interval")
            .get_parameter_value()
            .get<double>();
    double map_cloud_update_interval = this->get_parameter("map_cloud_update_interval")
                                           .get_parameter_value()
                                           .get<double>();
    std::string optimization_type = this->get_parameter("optimization_type")
                                        .get_parameter_value()
                                        .get<std::string>();
    
    if (optimization_type == "GLOBAL") {
      ongoing_optimization_class = optimization_class::GLOBAL;
    } else {
      ongoing_optimization_class = optimization_class::GLOBAL_LOCAL;
    }

    callback_group_opt_timer =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    callback_keyframe_timer =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    callback_map_pub_timer =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    keyframe_timer = this->create_wall_timer(
        std::chrono::milliseconds(int(keyframe_timer_update_interval)), 
        std::bind(&SGraphsNode::keyframe_update_timer_callback, this),
        callback_keyframe_timer);

    anchor_node = nullptr;
    anchor_edge = nullptr;
    // one time timer to initialize the classes with the current node obj
    main_timer = this->create_wall_timer(
             std::chrono::seconds(int(main_timer_interval)),
              std::bind(&SGraphsNode::init_subclass, this));
              
    if (save_info) {
      // check_create_file(slam_filename, slam_ofs);
      check_create_file(slam_loop_filename, slam_loop_ofs);
    }
  } // Node declaration end

 private:
  bool check_create_file(std::string filename, std::ofstream& file){
    if (std::filesystem::exists(filename)) {
        std::string backupFilename = filename + "_bak";
        try {
            std::filesystem::copy(filename, backupFilename, std::filesystem::copy_options::overwrite_existing);
            std::cout << "Backup file created: " << backupFilename << std::endl;
        } catch (const std::filesystem::filesystem_error& e) {
            std::cerr << "File copy error: " << e.what() << std::endl;
            return false;
        }
    }
    file.open(filename); //, std::ofstream::out | std::ofstream::app);
    if (!file) {
        std::cerr << "Failed to create file: " << filename << std::endl;
        return false;
    }
    file.close();
    return true;
  }
  
  void updateFile(const std::string &filename, const int &cell_num, const double &value){
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        throw std::runtime_error("Cannot open input file");
    }

    std::vector<std::string> rows;
    std::string line;
    while (std::getline(ifs, line)) {
        rows.push_back(line);  // Store each line in the vector
    }
    ifs.close();
    if (rows.empty()) {
        throw std::runtime_error("Input file is empty");
    }

    std::istringstream iss(rows[0]);
    std::ostringstream modified_row;
    std::string cell;
    size_t index = 0;
    while (std::getline(iss, cell, ',')) {
        if (index == static_cast<size_t>(cell_num)) {
            modified_row << std::to_string(std::stod(cell) + value) << ",";
        } else {
            modified_row << cell << ",";
        }
        ++index;
    }
    rows[0] = modified_row.str();  // Update the first row

    std::ofstream ofs(filename, std::ofstream::out | std::ofstream::trunc);
    if (!ofs.is_open()) {
        throw std::runtime_error("Cannot open output file");
    }
    for (const auto& row : rows) {
        ofs << row << "\n";
    }

    ofs.close();
  }

  void updateFile_loop(const std::string &filename, 
                  const double &time, const std::vector<Loop::Ptr> &loops, const int &loop_cnt){
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        throw std::runtime_error("Cannot open input file");
    }

    std::vector<std::string> rows;
    std::string line;
    while (std::getline(ifs, line)) {
        rows.push_back(line);  // Store each line in the vector
    }
    ifs.close();
    if (rows.empty()) {
        throw std::runtime_error("Input file is empty");
    }

    std::istringstream iss(rows[0]);
    std::ostringstream modified_row;
    std::vector<std::string> row;
    std::string cell;
    // Parse the first row into a vector
    while (std::getline(iss, cell, ',')) {
        row.push_back(cell);
    }
    // Modify the first row's output format
    modified_row << row[0] << ","                           // Use the first column as-is
                 << std::to_string(std::stod(row[1]) + time) << "," // Update the second column
                 << std::to_string(loop_cnt);              // Add loop_cnt as the third column

    rows[0] = modified_row.str();  // Update the first row in the vector

    std::ofstream ofs(filename, std::ofstream::out | std::ofstream::trunc);
    if (!ofs.is_open()) {
        throw std::runtime_error("Cannot open output file");
    }
    for (const auto& row : rows) {
        ofs << row << "\n";
    }
    ofs.close();
    
    ofs.open(filename, std::ofstream::out | std::ofstream::app);
    ofs << std::fixed << std::setprecision(2);
    for (auto &loop : loops){
      ofs <<std::to_string(loop->key1->stamp.seconds())<<","<<std::to_string(loop->key2->stamp.seconds())<<std::endl;
    }
    ofs.close();
  }
  
  void declare_ros_params() {
    this->declare_parameter("filtered_points", "filtered_points");
    this->declare_parameter("map_frame_id", "map");
    this->declare_parameter("odom_frame_id", "odom");
    this->declare_parameter("map_cloud_resolution", 0.05);
    this->declare_parameter("wait_trans_odom2map", false);
    this->declare_parameter("use_map2map_transform", false);
    this->declare_parameter("color_r", 0.0);
    this->declare_parameter("color_g", 0.0);
    this->declare_parameter("color_b", 0.0);

    this->declare_parameter("max_keyframes_per_update", 10);
    this->declare_parameter("gps_time_offset", 0);
    this->declare_parameter("gps_edge_stddev_xy", 10000.0);
    this->declare_parameter("gps_edge_stddev_z", 10.0);

    this->declare_parameter("imu_time_offset", 0);
    this->declare_parameter("enable_imu_orientation", false);
    this->declare_parameter("enable_imu_acceleration", false);
    this->declare_parameter("imu_orientation_edge_stddev", 0.1);
    this->declare_parameter("imu_acceleration_edge_stddev", 3.0);

    this->declare_parameter("distance_thresh", 5.0);
    this->declare_parameter("accum_distance_thresh", 8.0);
    this->declare_parameter("min_edge_interval", 5.0);
    this->declare_parameter("fitness_score_max_range",
                            std::numeric_limits<double>::max());
    this->declare_parameter("fitness_score_thresh", 0.5);
    this->declare_parameter("keyframe_matching_threshold", 0.1);

    this->declare_parameter("registration_method", "NDT_OMP");
    this->declare_parameter("reg_num_threads", 0);
    this->declare_parameter("reg_transformation_epsilon", 0.01);
    this->declare_parameter("reg_maximum_iterations", 64);
    this->declare_parameter("reg_max_correspondence_distance", 2.5);
    this->declare_parameter("reg_correspondence_randomness", 20);
    this->declare_parameter("reg_resolution", 1.0);
    this->declare_parameter("reg_use_reciprocal_correspondences", false);
    this->declare_parameter("reg_max_optimizer_iterations", 20);
    this->declare_parameter("reg_nn_search_method", "DIRECT7");

    this->declare_parameter("use_const_inf_matrix", false);
    this->declare_parameter("const_stddev_x", 0.5);
    this->declare_parameter("const_stddev_q", 0.1);

    this->declare_parameter("var_gain_a", 20.0);
    this->declare_parameter("min_stddev_x", 0.1);
    this->declare_parameter("max_stddev_x", 5.0);
    this->declare_parameter("min_stddev_q", 0.05);
    this->declare_parameter("max_stddev_q", 0.2);

    this->declare_parameter("keyframe_delta_trans", 2.0);
    this->declare_parameter("keyframe_delta_angle", 2.0);
    this->declare_parameter("keyframe_window_size", 1);
    this->declare_parameter("fix_first_node_adaptive", true);

    this->declare_parameter("optimization_window_size", 10);
    this->declare_parameter("extract_planar_surfaces", true);
    this->declare_parameter("constant_covariance", true);
    this->declare_parameter("use_parallel_plane_constraint", false);
    this->declare_parameter("use_perpendicular_plane_constraint", false);
    this->declare_parameter("g2o_solver_num_iterations", 1024);
    this->declare_parameter("g2o_solver_type", "lm_var");

    this->declare_parameter("min_seg_points", 100);
    this->declare_parameter("min_horizontal_inliers", 500);
    this->declare_parameter("min_vertical_inliers", 100);
    this->declare_parameter("use_euclidean_filter", true);
    this->declare_parameter("use_shadow_filter", false);
    this->declare_parameter("plane_extraction_frame_id", "base_link");
    this->declare_parameter("plane_visualization_frame_id", "base_link_elevated");

    this->declare_parameter("use_point_to_plane", false);
    this->declare_parameter("plane_information", 0.01);
    this->declare_parameter("plane_dist_threshold", 0.15);
    this->declare_parameter("plane_points_dist", 0.5);
    this->declare_parameter("min_plane_points", 100);

    this->declare_parameter("infinite_room_information", 1.0); ///
    this->declare_parameter("infinite_room_dist_threshold", 1.0);
    this->declare_parameter("room_information", 0.01);
    this->declare_parameter("room_dist_threshold", 1.0);
    this->declare_parameter("dupl_plane_matching_information", 0.01);

    this->declare_parameter("points_topic", "velodyne_points");
    this->declare_parameter("enable_gps", false);
    this->declare_parameter("graph_update_interval", 3.0);
    this->declare_parameter("keyframe_timer_update_interval", 3.0);
    this->declare_parameter("map_cloud_update_interval", 3.0);
    this->declare_parameter("main_timer_interval", 1.0);
    this->declare_parameter("optimization_type", "GLOBAL");
    //////////////
    this->declare_parameter("bag_duration", 100.0);
    this->declare_parameter("save_info", false);
    this->declare_parameter("save_timings", false);
    this->declare_parameter("output_file", "");
    this->declare_parameter("use_old_ablation", false);
  }

  void init_subclass() {
    covisibility_graph = std::make_shared<GraphSLAM>(
        this->get_parameter("g2o_solver_type").get_parameter_value().get<std::string>(),
        this->get_parameter("save_timings").get_parameter_value().get<bool>());
    compressed_graph = std::make_unique<GraphSLAM>(
        this->get_parameter("g2o_solver_type").get_parameter_value().get<std::string>(),
        this->get_parameter("save_timings").get_parameter_value().get<bool>());
    visualization_graph = std::make_unique<GraphSLAM>();
    keyframe_updater = std::make_unique<KeyframeUpdater>(shared_from_this());
    plane_analyzer = std::make_unique<PlaneAnalyzer>(shared_from_this());
    loop_mapper = std::make_unique<LoopMapper>(shared_from_this());
    loop_detector = std::make_unique<LoopDetector>(shared_from_this());
    map_cloud_generator = std::make_unique<MapCloudGenerator>();
    inf_calclator = std::make_unique<InformationMatrixCalculator>(shared_from_this());
    nmea_parser = std::make_unique<NmeaSentenceParser>();
    plane_mapper = std::make_unique<PlaneMapper>(shared_from_this());
    inf_room_mapper = std::make_unique<OldInfiniteRoomMapper>(shared_from_this());
    finite_room_mapper = std::make_unique<OldFiniteRoomMapper>(shared_from_this());
    floor_mapper = std::make_unique<FloorMapper>();
    graph_visualizer = std::make_unique<GraphVisualizer>(shared_from_this());
    keyframe_mapper = std::make_unique<KeyframeMapper>(shared_from_this());
    gps_mapper = std::make_unique<GPSMapper>(shared_from_this());
    imu_mapper = std::make_unique<IMUMapper>(shared_from_this());
    graph_publisher = std::make_unique<GraphPublisher>();
    wall_mapper = std::make_unique<WallMapper>(shared_from_this());
    room_graph_generator = std::make_unique<RoomGraphGenerator>(shared_from_this());
    infinite_room_graph_generator = std::make_unique<InfiniteRoomGraphGenerator>(shared_from_this());

    main_timer->cancel();
  }

 private:
  /**
  * @brief receive the raw odom msg to publish the corrected odom after s
  *
  */
  void raw_odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    odom_time = odom_msg->header.stamp.sec+odom_msg->header.stamp.nanosec*1e-9;

    // if (save_info){
    //   slam_loop_ofs.open(slam_loop_filename,
    //                     std::ofstream::out | std::ofstream::app);
    //   slam_loop_ofs << std::fixed << std::setprecision(2);
    //   slam_loop_ofs << odom_time <<","
    //       <<odom_msg->pose.pose.position.x<<","<<odom_msg->pose.pose.position.y<<","<<odom_msg->pose.pose.position.z<<","
    //       <<odom_msg->pose.pose.orientation.x<<","<<odom_msg->pose.pose.orientation.y<<","<<odom_msg->pose.pose.orientation.z<<","<<odom_msg->pose.pose.orientation.w<<"\n";
    //   slam_loop_ofs.close();
    // }
    
    Eigen::Isometry3d odom = odom2isometry(odom_msg);
    geometry_msgs::msg::TransformStamped odom2map_transform;
    Eigen::Isometry3d map2map_trans(trans_map2map.cast<double>());
    Eigen::Matrix4f odom_corrected;
    if (use_map2map_transform) {
      Eigen::Isometry3d odom_trans = map2map_trans * odom;
      odom_corrected = trans_odom2map * odom_trans.matrix().cast<float>();
    } else {
      odom_corrected = trans_odom2map * odom.matrix().cast<float>();
    }
    // std::cout<<"trans_odom2map "<<trans_odom2map.matrix() << std::endl;
    geometry_msgs::msg::PoseStamped pose_stamped_corrected =
        matrix2PoseStamped(odom_msg->header.stamp, odom_corrected, map_frame_id);
    publish_corrected_odom(pose_stamped_corrected);

    if (save_info){
      slam_loop_ofs.open(slam_loop_filename,
                        std::ofstream::out | std::ofstream::app);
      slam_loop_ofs << std::fixed << std::setprecision(2);
      slam_loop_ofs << odom_time <<","
          <<pose_stamped_corrected.pose.position.x<<","<<pose_stamped_corrected.pose.position.y<<","<<pose_stamped_corrected.pose.position.z<<","
          <<pose_stamped_corrected.pose.orientation.x<<","<<pose_stamped_corrected.pose.orientation.y<<","<<pose_stamped_corrected.pose.orientation.z<<","<<pose_stamped_corrected.pose.orientation.w<<"\n";
      slam_loop_ofs.close();
    }
    
    if (use_map2map_transform) {
      Eigen::Matrix4f current_map2map_trans = map2map_trans.matrix().cast<float>();
      odom2map_transform =
          matrix2transform(odom_msg->header.stamp,
                          trans_odom2map * map2map_trans.matrix().cast<float>(),
                          map_frame_id,
                          odom_frame_id);
    } else {
      odom2map_transform = matrix2transform(
          odom_msg->header.stamp, trans_odom2map, map_frame_id, odom_frame_id);
    }
    odom2map_broadcaster->sendTransform(odom2map_transform);
  }

  /**
  * @brief receive the raw pointcloud
  *
  * @param cloud_msg
  */
  void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    cloud_queue_mutex.lock();
    cloud_queue.push_back(cloud_msg);
    cloud_queue_mutex.unlock();
  }

  /**
  * @brief receive the initial transform between map and odom frame
  * @param map2odom_pose_msg
  */
  void init_map2odom_pose_callback(
      geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg) {
    if (got_trans_odom2map) return;

    Eigen::Matrix3f mat3 = Eigen::Quaternionf(pose_msg->pose.orientation.w,
                                              pose_msg->pose.orientation.x,
                                              pose_msg->pose.orientation.y,
                                              pose_msg->pose.orientation.z)
                              .toRotationMatrix();

    trans_odom2map.block<3, 3>(0, 0) = mat3;
    trans_odom2map(0, 3) = pose_msg->pose.position.x;
    trans_odom2map(1, 3) = pose_msg->pose.position.y;
    trans_odom2map(2, 3) = pose_msg->pose.position.z;

    if (trans_odom2map.isIdentity())
      return;
    else {
      got_trans_odom2map = true;
    }
  }

  /**
  * @brief receive the between loaded and new map frame
  * @param map2odom_pose_msg
  */
  void map2map_transform_callback(
      const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) {
    Eigen::Matrix3f mat3 = Eigen::Quaternionf(pose_msg->pose.orientation.w,
                                              pose_msg->pose.orientation.x,
                                              pose_msg->pose.orientation.y,
                                              pose_msg->pose.orientation.z)
                              .toRotationMatrix();
    trans_map2map.setIdentity();
    trans_map2map.block<3, 3>(0, 0) = mat3;
    trans_map2map(0, 3) = pose_msg->pose.position.x;
    trans_map2map(1, 3) = pose_msg->pose.position.y;
    trans_map2map(2, 3) = pose_msg->pose.position.z;
  }

  void cloud_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg,
                      const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    if (!odom_msg) {
        RCLCPP_ERROR(this->get_logger(), "Odometry message is null");
        return;
    } if (!cloud_msg) {
        RCLCPP_ERROR(this->get_logger(), "PointCloud2 message is null");
        return;
    } if (!keyframe_updater){
        RCLCPP_ERROR(this->get_logger(), "keyframe_updater is null");
        return;
    } 

    const rclcpp::Time& stamp = cloud_msg->header.stamp;
    Eigen::Isometry3d odom = odom2isometry(odom_msg);

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    if (base_frame_id.empty()) {
      base_frame_id = cloud_msg->header.frame_id;
    }

    if (!keyframe_updater->update(odom)) {
      std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
      if (keyframe_queue.empty()) {
        std_msgs::msg::Header read_until;
        read_until.stamp = stamp + rclcpp::Duration(10, 0);
        read_until.frame_id = points_topic;
        read_until_pub->publish(read_until);
        read_until.frame_id = "/filtered_points";
        read_until_pub->publish(read_until);
      }

      return;
    }

    double accum_d = keyframe_updater->get_accum_distance();
    if (use_map2map_transform) {
      Eigen::Isometry3d map2map_trans(trans_map2map.cast<double>());
      Eigen::Quaterniond quaternion(map2map_trans.rotation());
      Eigen::Isometry3d odom_trans = map2map_trans * odom;
      Eigen::Quaterniond odom_quaternion(odom_trans.rotation());

      KeyFrame::Ptr keyframe(new KeyFrame(stamp, odom_trans, accum_d, cloud));
      std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
      keyframe_queue.push_back(keyframe);
    } else {
      KeyFrame::Ptr keyframe(new KeyFrame(stamp, odom, accum_d, cloud));
      std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
      keyframe_queue.push_back(keyframe);
    }
  }

  /**
  * @brief this method adds all the keyframes in #keyframe_queue to the pose graph
  * (odometry edges)
  * @return if true, at least one keyframe was added to the pose graph
  */
  bool flush_keyframe_queue() {
    if (keyframe_queue.empty()) {
      // std::cout << "keyframe_queue is empty " << std::endl;
      return false;
    }

    trans_odom2map_mutex.lock();
    Eigen::Isometry3d odom2map(trans_odom2map.cast<double>());
    trans_odom2map_mutex.unlock();

    graph_mutex.lock();
    int num_processed = keyframe_mapper->map_keyframes(covisibility_graph,
                                                      odom2map,
                                                      keyframe_queue,
                                                      keyframes,
                                                      new_keyframes,
                                                      anchor_node,
                                                      anchor_edge,
                                                      keyframe_hash);
    graph_mutex.unlock();


    std_msgs::msg::Header read_until;
    read_until.stamp = keyframe_queue[num_processed]->stamp + rclcpp::Duration(10, 0);
    read_until.frame_id = points_topic;
    read_until_pub->publish(read_until);
    read_until.frame_id = "/filtered_points";
    read_until_pub->publish(read_until);

    std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
    keyframe_queue.erase(keyframe_queue.begin(),
                        keyframe_queue.begin() + num_processed + 1);

    return true;
  }

  void nmea_callback(const nmea_msgs::msg::Sentence::SharedPtr nmea_msg) {
    GPRMC grmc = nmea_parser->parse(nmea_msg->sentence);

    if (grmc.status != 'A') {
      return;
    }

    geographic_msgs::msg::GeoPointStamped::SharedPtr gps_msg(
        new geographic_msgs::msg::GeoPointStamped());
    gps_msg->header = nmea_msg->header;
    gps_msg->position.latitude = grmc.latitude;
    gps_msg->position.longitude = grmc.longitude;
    gps_msg->position.altitude = NAN;

    gps_callback(gps_msg);
  }

  void navsat_callback(const sensor_msgs::msg::NavSatFix::SharedPtr navsat_msg) {
    geographic_msgs::msg::GeoPointStamped::SharedPtr gps_msg(
        new geographic_msgs::msg::GeoPointStamped());
    gps_msg->header = navsat_msg->header;
    gps_msg->position.latitude = navsat_msg->latitude;
    gps_msg->position.longitude = navsat_msg->longitude;
    gps_msg->position.altitude = navsat_msg->altitude;

    gps_callback(gps_msg);
  }

  /**
  * @brief received gps data is added to #gps_queue
  * @param gps_msg
  */
  void gps_callback(const geographic_msgs::msg::GeoPointStamped::SharedPtr gps_msg) {
    std::lock_guard<std::mutex> lock(gps_queue_mutex);
    rclcpp::Time(gps_msg->header.stamp) += rclcpp::Duration(gps_time_offset, 0);
    gps_queue.push_back(gps_msg);
  }

  /**
  * @brief
  * @return
  */
  bool flush_gps_queue() {
    std::lock_guard<std::mutex> lock(gps_queue_mutex);

    if (keyframes.empty() || gps_queue.empty()) {
      return false;
    }

    return gps_mapper->map_gps_data(covisibility_graph, gps_queue, keyframes);
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
    if (!enable_imu_orientation && !enable_imu_acceleration) {
      return;
    }

    std::lock_guard<std::mutex> lock(imu_queue_mutex);
    rclcpp::Time(imu_msg->header.stamp) += rclcpp::Duration(imu_time_offset, 0);
    imu_queue.push_back(imu_msg);
  }

  bool flush_imu_queue() {
    std::lock_guard<std::mutex> lock(imu_queue_mutex);
    if (keyframes.empty() || imu_queue.empty() || base_frame_id.empty()) {
      return false;
    }

    return imu_mapper->map_imu_data(
        covisibility_graph, tf_buffer, imu_queue, keyframes, base_frame_id);
  }

  /**
  * @brief this methods adds all the data in the queues to the pose graph, and then
  * optimizes the pose graph
  * @param event
  */
  void keyframe_update_timer_callback() {

    auto t1 = rclcpp::Clock{}.now();
    // add keyframes and floor coeffs in the queues to the pose graph
    bool keyframe_updated = flush_keyframe_queue();

    if (!keyframe_updated) {
      std_msgs::msg::Header read_until;
      read_until.stamp = this->now() + rclcpp::Duration(30, 0);
      read_until.frame_id = points_topic;
      read_until_pub->publish(read_until);
      read_until.frame_id = "/filtered_points";
      read_until_pub->publish(read_until);
    }

    if (!keyframe_updated & !flush_gps_queue() & !flush_imu_queue()) {
      return;
    }

    // loop detection
    std::vector<Loop::Ptr> loops = loop_detector->detect(keyframes, new_keyframes, *covisibility_graph);

    if (loops.size() > 0) {
      loop_cnt = loop_cnt+loops.size();
      std::cout<<"loop_cnt "<<loop_cnt<<std::endl;
      loop_found = true;
      loop_mapper->add_loops(covisibility_graph, loops, graph_mutex);
    }

    graph_mutex.lock();
    std::transform(new_keyframes.begin(),
                  new_keyframes.end(),
                  std::inserter(keyframes, keyframes.end()),
                  [](const KeyFrame::Ptr& k) { return std::make_pair(k->id(), k); });

    new_keyframes.clear();
    graph_mutex.unlock();

    // move the first node anchor position to the current estimate of the first node
    // pose so the first node moves freely while trying to stay around the origin
    if (anchor_node && this->get_parameter("fix_first_node_adaptive")
                          .get_parameter_value()
                          .get<bool>()) {
      Eigen::Isometry3d anchor_target =
          static_cast<g2o::VertexSE3*>(anchor_edge->vertices()[1])->estimate();
      anchor_node->setEstimate(anchor_target);
    }

    std::vector<KeyFrameSnapshot::Ptr> snapshot(keyframes.size());
    std::transform(keyframes.begin(),
                  keyframes.end(),
                  snapshot.begin(),
                  [=](const std::pair<int, KeyFrame::Ptr>& k) {
                    return std::make_shared<KeyFrameSnapshot>(k.second);
                  });
    keyframes_snapshot.swap(snapshot);

    for (const auto& keyframe_snapshot : keyframes_snapshot) {
      if (!keyframe_snapshot->k_marginalized)
        keyframes_snapshot_queue.push(keyframe_snapshot);
    }

    for (const auto& keyframe : keyframes) {
      complete_keyframes_queue.push(keyframe.second);
    }

    std::vector<VerticalPlanes> current_x_planes(x_vert_planes.size());
    std::transform(x_vert_planes.begin(),
                  x_vert_planes.end(),
                  current_x_planes.begin(),
                  [](const std::pair<int, VerticalPlanes>& x_plane) {
                    return VerticalPlanes(x_plane.second, true);
                  });
    x_planes_snapshot.swap(current_x_planes);

    std::vector<VerticalPlanes> current_y_planes(y_vert_planes.size());
    std::transform(y_vert_planes.begin(),
                  y_vert_planes.end(),
                  current_y_planes.begin(),
                  [](const std::pair<int, VerticalPlanes>& y_plane) {
                    return VerticalPlanes(y_plane.second, true);
                  });
    y_planes_snapshot.swap(current_y_planes);

    std::vector<HorizontalPlanes> current_hort_planes(hort_planes.size());
    std::transform(hort_planes.begin(),
                  hort_planes.end(),
                  current_hort_planes.begin(),
                  [](const std::pair<int, HorizontalPlanes>& hort_plane) {
                    return HorizontalPlanes(hort_plane.second, true);
                  });
    hort_planes_snapshot.swap(current_hort_planes);

    std::vector<InfiniteRooms> curent_x_inf_rooms(x_infinite_rooms.size());
    std::transform(x_infinite_rooms.begin(),
                  x_infinite_rooms.end(),
                  curent_x_inf_rooms.begin(),
                  [](const std::pair<int, InfiniteRooms>& x_inf_room) {
                    return InfiniteRooms(x_inf_room.second, true);
                  });
    x_inf_rooms_snapshot.swap(curent_x_inf_rooms);

    std::vector<InfiniteRooms> curent_y_inf_rooms(y_infinite_rooms.size());
    std::transform(y_infinite_rooms.begin(),
                  y_infinite_rooms.end(),
                  curent_y_inf_rooms.begin(),
                  [](const std::pair<int, InfiniteRooms>& y_inf_room) {
                    return InfiniteRooms(y_inf_room.second, true);
                  });
    y_inf_rooms_snapshot.swap(curent_y_inf_rooms);

    std::vector<Rooms> curent_rooms(rooms_vec.size());
    std::transform(
        rooms_vec.begin(),
        rooms_vec.end(),
        curent_rooms.begin(),
        [](const std::pair<int, Rooms>& room) { return Rooms(room.second, true); });
    rooms_vec_snapshot.swap(curent_rooms);

    std::vector<Floors> current_floors(floors_vec.size());
    std::transform(floors_vec.begin(),
                  floors_vec.end(),
                  current_floors.begin(),
                  [](const std::pair<int, Floors>& floor) { return floor.second; });
    floors_vec_snapshot.swap(current_floors);
  }

  /**
  * @brief publish odom corrected pose and path
  */
  void publish_corrected_odom(geometry_msgs::msg::PoseStamped pose_stamped_corrected) {
    nav_msgs::msg::Odometry odom_corrected;
    odom_corrected.pose.pose = pose_stamped_corrected.pose;
    odom_corrected.header = pose_stamped_corrected.header;
    odom_corrected_pub->publish(odom_corrected);

    nav_msgs::msg::Path path_stamped_corrected;
    path_stamped_corrected.header = pose_stamped_corrected.header;
    odom_path_vec.push_back(pose_stamped_corrected);
    path_stamped_corrected.poses = odom_path_vec;

    odom_pose_corrected_pub->publish(pose_stamped_corrected);
    odom_path_corrected_pub->publish(path_stamped_corrected);
  }


 private:
  // ROS
  rclcpp::TimerBase::SharedPtr main_timer;
  rclcpp::TimerBase::SharedPtr optimization_timer;
  rclcpp::TimerBase::SharedPtr keyframe_timer;
  rclcpp::TimerBase::SharedPtr map_publish_timer;

  rclcpp::CallbackGroup::SharedPtr callback_group_subscriber;
  rclcpp::CallbackGroup::SharedPtr callback_group_publisher;

  rclcpp::CallbackGroup::SharedPtr callback_group_opt_timer;
  rclcpp::CallbackGroup::SharedPtr callback_keyframe_timer;
  rclcpp::CallbackGroup::SharedPtr callback_map_pub_timer;

  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry,
                                                          sensor_msgs::msg::PointCloud2>
      ApproxSyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync;

  rclcpp::Subscription<geographic_msgs::msg::GeoPointStamped>::SharedPtr gps_sub;
  rclcpp::Subscription<nmea_msgs::msg::Sentence>::SharedPtr nmea_sub;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_odom_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<situational_graphs_msgs::msg::CompleteRoomsData>::SharedPtr
      complete_room_data_sub;
  rclcpp::Subscription<situational_graphs_msgs::msg::RoomsData>::SharedPtr
      room_data_sub;
  rclcpp::Subscription<situational_graphs_msgs::msg::WallsData>::SharedPtr
      wall_data_sub;
  rclcpp::Subscription<situational_graphs_msgs::msg::RoomData>::SharedPtr
      floor_data_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr init_odom2map_sub,
      map_2map_transform_sub;

  std::mutex trans_odom2map_mutex;
  Eigen::Matrix4f trans_odom2map, trans_map2map;
  bool wait_trans_odom2map, got_trans_odom2map, use_map2map_transform;
  std::vector<geometry_msgs::msg::PoseStamped> odom_path_vec;
  std::string map_frame_id;
  std::string odom_frame_id;
  std::string cloud_frame_id;
  std::string points_topic;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr odom2map_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_corrected_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr odom_pose_corrected_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odom_path_corrected_pub;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr read_until_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub;
  rclcpp::Publisher<situational_graphs_msgs::msg::PlanesData>::SharedPtr map_planes_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_planes_point_pub;
  // rclcpp::Publisher<situational_graphs_msgs::msg::PlanesData>::SharedPtr map_hort_planes_pub;////////////
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_hort_planes_point_pub;/////
  rclcpp::Publisher<situational_graphs_msgs::msg::PlanesData>::SharedPtr all_map_planes_pub;
  rclcpp::Publisher<situational_graphs_reasoning_msgs::msg::Graph>::SharedPtr graph_pub;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom2map_broadcaster;

  rclcpp::Service<situational_graphs_msgs::srv::DumpGraph>::SharedPtr
      dump_service_server;
  rclcpp::Service<situational_graphs_msgs::srv::LoadGraph>::SharedPtr
      load_service_server;
  rclcpp::Service<situational_graphs_msgs::srv::SaveMap>::SharedPtr
      save_map_service_server;

  // odom queue
  std::mutex odom_queue_mutex;
  std::deque<nav_msgs::msg::Odometry::SharedPtr> odom_queue;

  // cloud queue
  std::mutex cloud_queue_mutex;
  std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> cloud_queue;

  // keyframe queue
  std::string base_frame_id;
  std::mutex keyframe_queue_mutex;
  std::deque<KeyFrame::Ptr> keyframe_queue;

  // gps queue
  int gps_time_offset;
  double gps_edge_stddev_xy;
  double gps_edge_stddev_z;
  boost::optional<Eigen::Vector3d> zero_utm;
  std::mutex gps_queue_mutex;
  std::deque<geographic_msgs::msg::GeoPointStamped::SharedPtr> gps_queue;

  // imu queue
  int imu_time_offset;
  bool enable_imu_orientation;
  double imu_orientation_edge_stddev;
  bool enable_imu_acceleration;
  double imu_acceleration_edge_stddev;
  std::mutex imu_queue_mutex;
  std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_queue;

  std::deque<int> room_local_graph_id_queue;
  int optimization_window_size;
  bool loop_found, duplicate_planes_found;
  bool global_optimization;
  int keyframe_window_size;
  bool extract_planar_surfaces;
  bool constant_covariance;
  double min_plane_points;
  double infinite_room_information;
  double room_information, plane_information;
  ///////////
  rclcpp::TimerBase::SharedPtr bag_finish_timer;
  bool save_timings, save_info;
  std::string output_file;
  std::string slam_filename, slam_loop_filename;
  double main_timer_interval;
  bool use_old_ablation;

  enum optimization_class : uint8_t {
    GLOBAL = 1,
    GLOBAL_LOCAL = 2,
  } ongoing_optimization_class;

  // vertical and horizontal planes
  std::unordered_map<int, VerticalPlanes> x_vert_planes,
      y_vert_planes;  // vertically segmented planes
  std::vector<VerticalPlanes> x_vert_planes_prior, y_vert_planes_prior;
  std::deque<std::pair<VerticalPlanes, VerticalPlanes>> dupl_x_vert_planes,
      dupl_y_vert_planes;  // vertically segmented planes
  std::unordered_map<int, HorizontalPlanes>
      hort_planes;  // horizontally segmented planes
  std::unordered_map<int, InfiniteRooms> x_infinite_rooms,
      y_infinite_rooms;                      // infinite_rooms segmented from planes
  std::unordered_map<int, Rooms> rooms_vec;  // rooms segmented from planes
  std::vector<Rooms> rooms_vec_prior;
  std::unordered_map<int, Floors> floors_vec;
  int prev_edge_count, curr_edge_count;

  std::vector<VerticalPlanes> x_planes_snapshot, y_planes_snapshot;
  std::vector<HorizontalPlanes> hort_planes_snapshot;
  std::vector<Rooms> rooms_vec_snapshot;
  std::vector<Floors> floors_vec_snapshot;
  std::vector<InfiniteRooms> x_inf_rooms_snapshot, y_inf_rooms_snapshot;

  std::vector<situational_graphs_msgs::msg::RoomData> recent_rooms; //////
  // room data queue
  std::mutex complete_room_data_queue_mutex, room_data_queue_mutex, floor_data_mutex;
  std::deque<situational_graphs_msgs::msg::RoomsData> room_data_queue;
  std::deque<situational_graphs_msgs::msg::CompleteRoomsData> complete_room_data_queue;
  std::deque<situational_graphs_msgs::msg::RoomData> floor_data_queue;

  // for map cloud generation
  std::atomic_bool graph_updated;
  double map_cloud_resolution;
  std::vector<KeyFrameSnapshot::Ptr> keyframes_snapshot;
  std::unique_ptr<MapCloudGenerator> map_cloud_generator;

  boost::lockfree::spsc_queue<KeyFrameSnapshot::Ptr> keyframes_snapshot_queue{1000};
  boost::lockfree::spsc_queue<KeyFrame::Ptr> complete_keyframes_queue{1000};
  std::vector<KeyFrame::Ptr> current_keyframes;
  std::vector<KeyFrameSnapshot::Ptr> current_keyframes_snapshot;

  std::mutex graph_mutex;
  int max_keyframes_per_update;
  std::deque<KeyFrame::Ptr> new_keyframes;

  g2o::VertexSE3* anchor_node;
  g2o::EdgeSE3* anchor_edge;
  std::map<int, KeyFrame::Ptr> keyframes;
  std::unordered_map<rclcpp::Time, KeyFrame::Ptr, RosTimeHash> keyframe_hash;

  std::shared_ptr<GraphSLAM> covisibility_graph;
  std::unique_ptr<GraphSLAM> compressed_graph;
  std::unique_ptr<GraphSLAM> visualization_graph;
  std::unique_ptr<LoopDetector> loop_detector;
  std::unique_ptr<LoopMapper> loop_mapper;
  std::unique_ptr<KeyframeUpdater> keyframe_updater;
  std::unique_ptr<PlaneAnalyzer> plane_analyzer;
  std::unique_ptr<NmeaSentenceParser> nmea_parser;
  std::unique_ptr<InformationMatrixCalculator> inf_calclator;
  std::unique_ptr<WallMapper> wall_mapper;
  std::unique_ptr<PlaneMapper> plane_mapper;
  std::unique_ptr<OldInfiniteRoomMapper> inf_room_mapper;
  std::unique_ptr<OldFiniteRoomMapper> finite_room_mapper;
  std::unique_ptr<FloorMapper> floor_mapper;
  std::unique_ptr<GraphVisualizer> graph_visualizer;
  std::unique_ptr<KeyframeMapper> keyframe_mapper;
  std::unique_ptr<GPSMapper> gps_mapper;
  std::unique_ptr<IMUMapper> imu_mapper;
  std::unique_ptr<GraphPublisher> graph_publisher;
  std::unique_ptr<RoomGraphGenerator> room_graph_generator;
  std::unique_ptr<InfiniteRoomGraphGenerator> infinite_room_graph_generator;

  std::map<int, s_graphs::KeyFrame::Ptr> current_room_keyframes; 
  int loop_cnt = 0;
  double odom_time;
  std::ofstream slam_ofs, slam_loop_ofs; ////
};

}  // namespace s_graphs

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor multi_executor;
  auto node = std::make_shared<s_graphs::SGraphsNode>();
  multi_executor.add_node(node);
  multi_executor.spin();
  rclcpp::shutdown();
  return 0;
}
