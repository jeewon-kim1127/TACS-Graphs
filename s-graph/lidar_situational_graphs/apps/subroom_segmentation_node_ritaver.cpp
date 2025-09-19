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

#include <math.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/angles.h>
#include <pcl/common/distances.h>
#include <pcl/common/io.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/shadowpoints.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ml/kmeans.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

#include <boost/format.hpp>
#include <cmath>
#include <iostream>
#include <s_graphs/common/plane_utils.hpp>
#include <string>
#include <algorithm>

#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "rclcpp/rclcpp.hpp"
#include "situational_graphs_msgs/msg/rooms_data.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "std_msgs/msg/bool.hpp"


#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"


#include <s_graphs/frontend/subroom_analyzer.hpp>

namespace s_graphs {

class SubRoomSegmentationNode : public rclcpp::Node {
 public:
  typedef pcl::PointXYZRGBNormal PointT;

  SubRoomSegmentationNode() : Node("room_segmentation_node") {
    this->initialize_params();
    this->init_ros();
    std::string ns = this->get_namespace();
    if (ns.length() > 1) {
      std::string ns_prefix = std::string(this->get_namespace()).substr(1);
      map_frame_id = ns_prefix + "/" + map_frame_id;
    }
  }

 private:
  void initialize_params() {
    prev_x_width = 0; prev_y_width = 0;
    this->declare_parameter("vertex_neigh_thres", 2);
    this->declare_parameter("save_timings", true);
    this->declare_parameter("output_file", "/root/data");
    this->declare_parameter("roomseg_timer", 5);

    room_analyzer_params params{
        this->get_parameter("vertex_neigh_thres").get_parameter_value().get<int>()};
    save_timings =
        this->get_parameter("save_timings").get_parameter_value().get<bool>();
    
    output_file = 
        this->get_parameter("output_file").get_parameter_value().get<std::string>();
    roomseg_timer = 
        this->get_parameter("roomseg_timer").get_parameter_value().get<int>();
    
    room_analyzer.reset(new SubRoomAnalyzer(params));
    cloud_cluster = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    tmp_cloud_cluster = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

    // if (save_timings) {
    //   time_recorder.open(output_file+"_time.csv");
    //   time_recorder << "#time \n";
    //   time_recorder.close();
    // }

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  
  }

  void init_ros() {
    corrected_odom_sub = 
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "s_graphs/odom_pose_corrected", 10, 
            std::bind(&SubRoomSegmentationNode::corrected_odom_callback,
                      this,
                       std::placeholders::_1));

    skeleton_graph_sub =
        this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "travelgseg/edges",
            1,
            std::bind(&SubRoomSegmentationNode::local_traversable_edges_callback,
                      this,
                      std::placeholders::_1));     //get local graphs
    // cluster_cloud_sub = 
    //     this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //           "complete_room_segmentation/current_cluster_cloud",
    //           1,
    //           std::bind(&SubRoomSegmentationNode::local_traversable_cloud_callback,
    //                   this,
    //                   std::placeholders::_1)); //get current traversable cloud points in global frame
    // cluster_cloud_marker_sub = 
    //     this->create_subscription<visualization_msgs::msg::MarkerArray>(
    //           "complete_room_segmentation/current_cloud_marker",
    //           1,
    //           std::bind(&SubRoomSegmentationNode::local_traversable_cloud_marker_callback,
    //                   this,
    //                   std::placeholders::_1));
    is_room_completed_sub = 
        this->create_subscription<std_msgs::msg::Bool>(
              "complete_room_segmentation/is_room_completed",
              1,
              std::bind(&SubRoomSegmentationNode::is_room_completed_callback,
                      this,
                      std::placeholders::_1));
    map_planes_sub =
        this->create_subscription<situational_graphs_msgs::msg::PlanesData>(
            "s_graphs/map_planes",
            100,
            std::bind(&SubRoomSegmentationNode::map_planes_callback,
                      this,
                      std::placeholders::_1));

    cluster_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "room_segmentation/cluster_cloud", 1);
    cluster_clouds_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "room_segmentation/cluster_clouds", 1);
    room_data_pub = this->create_publisher<situational_graphs_msgs::msg::RoomsData>(
        "room_segmentation/room_data", 1);
    room_centers_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "room_segmentation/room_centers", 1);

    // room_detection_timer = create_wall_timer(
    //     std::chrono::seconds(6), //milliseconds(500),
    //     std::bind(&SubRoomSegmentationNode::room_detection_callback, this));
  }

  template <typename T>
  bool contains(std::vector<T> vec, const T& elem) {
    bool result = false;
    if (find(vec.begin(), vec.end(), elem) != vec.end()) {
      result = true;
    }
    return result;
  }

  void corrected_odom_callback(
      const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg){
    // m_lock_odom.lock();
    odom_buf.push_back(*pose_msg);

    if (sqrt(pow(pose_msg->pose.position.x,2)+pow(pose_msg->pose.position.y,2))<0.6){
      std::cout<<"!!!!pass to cloud_cluster"<<std::endl;
      cloud_cluster->points.clear();
      for (auto &pt: tmp_cloud_cluster->points) cloud_cluster->points.push_back(pt); 
      tmp_cloud_cluster->points.clear();

      room_detection_callback();
    }

  }

  void room_detection_callback() {
    std::vector<situational_graphs_msgs::msg::PlaneData> current_x_vert_planes,
        current_y_vert_planes;
    flush_map_planes(current_x_vert_planes, current_y_vert_planes);

    if (current_x_vert_planes.empty() && current_y_vert_planes.empty()) {
      // RCLCPP_INFO(this->get_logger(), "Did not receive any mapped planes");
      return;
    }
    
    auto t1 = this->now();
    extract_rooms(current_x_vert_planes, current_y_vert_planes);
    auto t2 = this->now();
    // std::cout << "duration to extract clusters: " << boost::format("%.3f") % (t2 -
    // t1).seconds() << std::endl;
  //   if (save_timings) {
  //     time_recorder.open(output_file+"_time.csv",
  //                        std::ofstream::out | std::ofstream::app);
  //     time_recorder << std::to_string((t2 - t1).seconds()) + " \n";
  //     time_recorder.close();
  //   }
  }

  /**
   * @brief get the vertical planes in map frame
   * @param map_planes_msg
   */
  void map_planes_callback(
      const situational_graphs_msgs::msg::PlanesData::SharedPtr map_planes_msg) {
    std::lock_guard<std::mutex> lock(map_plane_mutex);
    x_vert_plane_queue.push_back(map_planes_msg->x_planes);
    y_vert_plane_queue.push_back(map_planes_msg->y_planes);
  }

  void flush_map_planes(
      std::vector<situational_graphs_msgs::msg::PlaneData>& current_x_vert_planes,
      std::vector<situational_graphs_msgs::msg::PlaneData>& current_y_vert_planes) {
    std::lock_guard<std::mutex> lock(map_plane_mutex);
    for (const auto& x_map_planes_msg : x_vert_plane_queue) {
      for (const auto& x_map_plane : x_map_planes_msg) {
        if (!contains(current_x_vert_planes, x_map_plane) ||
            current_x_vert_planes.empty()) {
          current_x_vert_planes.push_back(x_map_plane);
        } else {
          continue;
        }
      }
      // x_vert_plane_queue.pop_front();
    }

    for (const auto& y_map_planes_msg : y_vert_plane_queue) {
      for (const auto& y_map_plane : y_map_planes_msg) {
        if (!contains(current_y_vert_planes, y_map_plane) ||
            current_y_vert_planes.empty()) {
          current_y_vert_planes.push_back(y_map_plane);
        } else {
          continue;
        }
      }
      // y_vert_plane_queue.pop_front();
    }
  }

  void is_room_completed_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    // if (msg->data){
    //   cloud_cluster->points.clear();
    //   curr_cloud_marker.markers.clear();
    //   x_vert_plane_vec.clear();
    //   y_vert_plane_vec.clear();
    //   x_vert_plane_queue.clear();
    //   y_vert_plane_queue.clear();
    // }
  }
  
  /**
   *
   * @brief get the points from the skeleton graph for clusterting and identifying room
   * candidates
   * @param skeleton_graph_msg
   */
  // void local_traversable_cloud_marker_callback( 
  //       const visualization_msgs::msg::MarkerArray::SharedPtr marker_msg){
  //   for (auto& marker: marker_msg->markers)
  //     curr_cloud_marker.markers.push_back(marker);
    
  // }
  
  void change_local_marker2global_pcl(
    const visualization_msgs::msg::MarkerArray::SharedPtr skeleton_graph_msg,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    visualization_msgs::msg::MarkerArray& marker_arr){
  if (!cloud) return;
  if (skeleton_graph_msg->markers.empty()|| odom_buf.empty()) return;
  
  try {
    tf_local2global = tf_buffer->lookupTransform(       //tf: local(skeleton) -> global(odom)
              odom_buf.back().header.frame_id,
              skeleton_graph_msg->markers[0].header.frame_id,
              tf2::TimePointZero); 
    }catch (const tf2::TransformException& ex) {
    std::cout<< "Could not transform "<< skeleton_graph_msg->markers[0].header.frame_id<< " to "
                    <<odom_buf.front().header.frame_id<<std::endl;
    return;
  }
  Eigen::Affine3d transform = tf2::transformToEigen(tf_local2global);

  pcl::PointXYZRGB curr_position ;
  curr_position.x = odom_buf.back().pose.position.x;
  curr_position.y = odom_buf.back().pose.position.y;
  curr_position.z = odom_buf.back().pose.position.z;
 
  int robot_in_cloud = 0;
  for (const auto& single_graph : skeleton_graph_msg->markers) { //vertices, connected_vertices_, connected_edges_ .. several ns
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>());
    if (!marker_arr.markers.empty()) marker_arr.markers.clear();
    
    std::string vertex_string = "connected_edges_";
    size_t found = single_graph.ns.find(vertex_string);
    if (found != std::string::npos) {
      for (size_t i = 0; i < single_graph.points.size(); ++i) {
        pcl::PointXYZRGB pcl_point;
        pcl_point.x = single_graph.points[i].x;
        pcl_point.y = single_graph.points[i].y;
        pcl_point.z = single_graph.points[i].z;
        local_cloud_cluster->points.push_back(pcl_point);
      }

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_global_cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::transformPointCloud(*local_cloud_cluster, *tmp_global_cloud_cluster, transform);  
      // std::cout<<"skeleton_graph_msg->markers.size() "<<skeleton_graph_msg->markers.size()<<std::endl;
      room_analyzer->downsample_cloud_data(tmp_global_cloud_cluster);
      if (skeleton_graph_msg->markers.size()==1){
        visualization_msgs::msg::Marker marker;
        for (auto &pt: tmp_global_cloud_cluster->points){
            cloud->points.push_back(pt);

            geometry_msgs::msg::Point p;
            p.x = pt.x; p.y = pt.y; p.z = pt.z;
            marker.points.push_back(p);
            std_msgs::msg::ColorRGBA color;
            color.r = 1; color.g = 1; color.b = 1;
            color.a = 1.0;  // Fully opaque
            marker.colors.push_back(color);
        }
        marker_arr.markers.push_back(marker);
        robot_in_cloud = 1;
        break;
      }
      else if (pcl::isPointIn2DPolygon(curr_position, *tmp_global_cloud_cluster)) {
        visualization_msgs::msg::Marker marker;
        for (auto &pt: tmp_global_cloud_cluster->points){
            cloud->points.push_back(pt);
            geometry_msgs::msg::Point p;
            p.x = pt.x; p.y = pt.y; p.z = pt.z;
            marker.points.push_back(p);
            std_msgs::msg::ColorRGBA color;
            color.r = 1; color.g = 1; color.b = 1;
            color.a = 1.0;  // Fully opaque
            marker.colors.push_back(color);
        }
        marker_arr.markers.push_back(marker);
        robot_in_cloud = 1;
        // break;
      }
    }
  }
  }

  void local_traversable_edges_callback(
    const visualization_msgs::msg::MarkerArray::SharedPtr skeleton_graph_msg) {
    std::lock_guard<std::mutex> lock(cloud_mutex);
    cloud_passed = 0;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr curr_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    change_local_marker2global_pcl(skeleton_graph_msg, curr_cloud, curr_cloud_marker);
   
    if (curr_cloud->points.size()<10) {
      prev_x_width = 0;
      prev_y_width = 0;
      return ; 
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr x_filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr y_filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass_x , pass_y;
    pcl::PointXY p1, p2;

    auto cloud_in = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    cloud_in->points = tmp_cloud_cluster->points; //tmp_cloud_cluster->points; 
    geometry_msgs::msg::PoseStamped curr_odom = odom_buf.back();

    double curr_x = curr_odom.pose.position.x;
    pass_x.setInputCloud(cloud_in);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(curr_x-0.1, curr_x+0.1);
    pass_x.filter(*x_filtered_cloud);
    if (x_filtered_cloud->points.empty()) {
      prev_x_width = 0; prev_y_width = 0;
      for (auto &pt: curr_cloud->points)
        tmp_cloud_cluster->points.push_back(pt); //stacked curr_cloud
      return ; 
    }
    room_analyzer->extract_cluster_endpoints(x_filtered_cloud, p1, p2);
    double y_width = p2.y - p1.y;

    double curr_y = curr_odom.pose.position.y;
    pass_y.setInputCloud(cloud_in);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(curr_y-0.1, curr_y+0.1);
    pass_y.filter(*y_filtered_cloud);
    if (y_filtered_cloud->points.empty()) {
      prev_x_width = 0; prev_y_width = 0;
      for (auto &pt: curr_cloud->points)
        tmp_cloud_cluster->points.push_back(pt); //stacked curr_cloud
      return ; 
    }
    room_analyzer->extract_cluster_endpoints(y_filtered_cloud, p1, p2);
    double x_width = p2.x - p1.x;
    // std::cout<<"x_width "<<x_width<<" y_width "<<y_width<<std::endl;
    if (prev_x_width == 0 && prev_y_width ==0 ) {
      prev_x_width = x_width;
      prev_y_width = y_width;
    }
    // std::cout<<"x_width "<<x_width<<" y_width "<<y_width<<std::endl;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZRGB>);
    float hull_area;
    double change_th = 1.0;
    double xy_imbalance = 1.0;
    if(y_width>x_width && abs(x_width-y_width)>xy_imbalance && abs(x_width-prev_x_width)<change_th){ // y parallel plane(x vert)
      // std::cout<<"passing x vert plane"<<std::endl;
      for (auto &pt: curr_cloud->points)
        tmp_cloud_cluster->points.push_back(pt); //stacked curr_cloud
    }
    else if (y_width<x_width && abs(x_width-y_width)>xy_imbalance && abs(y_width-prev_y_width)<change_th){
      // std::cout<<"passing y vert plane"<<std::endl;
      for (auto &pt: curr_cloud->points)
        tmp_cloud_cluster->points.push_back(pt); //stacked curr_cloud
    }
    else {
      std::cout<<"!!!!pass to cloud_cluster"<<std::endl;
      cloud_passed = 1;      
      cloud_cluster->points.clear();
      for (auto &pt: tmp_cloud_cluster->points) cloud_cluster->points.push_back(pt); 
      tmp_cloud_cluster->points.clear();
      room_detection_callback();
    }

    prev_x_width = x_width;
    prev_y_width = y_width;
  }

  /**
   * @brief extract clusters with its centers from the skeletal cloud
   *
   */
  void extract_rooms(
      std::vector<situational_graphs_msgs::msg::PlaneData> current_x_vert_planes,
      std::vector<situational_graphs_msgs::msg::PlaneData> current_y_vert_planes) {
    int room_cluster_counter = 0;
    visualization_msgs::msg::MarkerArray refined_skeleton_marker_array;
    std::vector<situational_graphs_msgs::msg::RoomData> room_candidates_vec;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_visualizer(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull_visualizer(
        new pcl::PointCloud<pcl::PointXYZRGB>);

      if (cloud_cluster->points.size() < 10) {
        return;
      }
      std::cout<<"planes vectors "<<current_x_vert_planes.size()<<" "<<current_y_vert_planes.size()<<std::endl;
      RoomInfo room_info = {current_x_vert_planes, current_y_vert_planes, cloud_cluster};

      pcl::PointXY p1, p2;
      situational_graphs_msgs::msg::PlaneData x_plane1;
      x_plane1.nx = 0; x_plane1.ny = 0; x_plane1.nz = 0;
      situational_graphs_msgs::msg::PlaneData x_plane2;
      x_plane2.nx = 0; x_plane2.ny = 0; x_plane2.nz = 0;
      situational_graphs_msgs::msg::PlaneData y_plane1;
      y_plane1.nx = 0; y_plane1.ny = 0; y_plane1.nz = 0;
      situational_graphs_msgs::msg::PlaneData y_plane2;
      y_plane2.nx = 0; y_plane2.ny = 0; y_plane2.nz = 0;
      RoomPlanes room_planes = {
          x_plane1, x_plane2, y_plane1, y_plane2, false, false, false, false};
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr sub_cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>());
      geometry_msgs::msg::Point room_length;

      bool found_planes = room_analyzer->extract_planes(room_info, cloud_cluster,
                              p1, p2, room_planes, sub_cloud_cluster, room_length);

      // std::cout<<"found_planes "<<found_planes<< " sub_cloud_cluster "<<sub_cloud_cluster->points.size()<<" room_length "<<room_length.x<<" "<<room_length.y<<std::endl;
      if (!found_planes) return;

      std::cout<<"input: "<<cloud_cluster->points.size()<<" "<<sub_cloud_cluster->points.size()<<" "
                <<room_length.x<<" "<<room_length.y<<std::endl;
      bool found_room = room_analyzer->perform_room_segmentation(cloud_cluster, p1, p2, room_planes, sub_cloud_cluster, room_length,
                            room_candidates_vec, curr_cloud_marker);
      std::cout<<"output: "<<room_candidates_vec.size()<<" "<<curr_cloud_marker.markers.size()<<std::endl;

      std::cout<<"===================================="<<std::endl;
      std::cout<<"found_room "<<found_room<<std::endl;
      std::cout<<"===================================="<<std::endl;
      for (int i = 0; i < cloud_cluster->points.size(); ++i) {
        cloud_visualizer->points.push_back(cloud_cluster->points[i]);
      }
    
    situational_graphs_msgs::msg::RoomsData room_candidates_msg;
    room_candidates_msg.header.stamp = this->now();
    room_candidates_msg.rooms = room_candidates_vec;
    room_data_pub->publish(room_candidates_msg);
    viz_room_centers(room_candidates_msg);

    sensor_msgs::msg::PointCloud2 cloud_cluster_msg;
    pcl::toROSMsg(*cloud_visualizer, cloud_cluster_msg);
    cloud_cluster_msg.header.stamp = this->now();
    cloud_cluster_msg.header.frame_id = map_frame_id;
    cluster_cloud_pub->publish(cloud_cluster_msg);

    // if (found_room) {
      cloud_cluster->points.clear();
      curr_cloud_marker.markers.clear();
      x_vert_plane_queue.clear();
      y_vert_plane_queue.clear();
    // }
  }

  void viz_room_centers(situational_graphs_msgs::msg::RoomsData room_vec) {
    visualization_msgs::msg::Marker room_marker;
    room_marker.pose.orientation.w = 1.0;
    room_marker.scale.x = 0.5;
    room_marker.scale.y = 0.5;
    room_marker.scale.z = 0.5;
    // plane_marker.points.resize(vert_planes.size());
    room_marker.header.frame_id = map_frame_id;
    room_marker.header.stamp = this->now();
    room_marker.ns = "rooms";
    room_marker.id = 0;
    room_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    room_marker.color.r = 1;
    // room_marker.color.g = 0.07;
    // room_marker.color.b = 0.0;
    // room_marker.color.a = 1;

    for (const auto& room : room_vec.rooms) {
      geometry_msgs::msg::Point point;
      point.x = room.room_center.position.x;
      point.y = room.room_center.position.y;
      point.z = 7.0;
      room_marker.points.push_back(point);

      std_msgs::msg::ColorRGBA point_color;
      if (room.x_planes.size() == 2 && room.y_planes.size() == 2) {
        point_color.r = 1;
        point_color.a = 1;
      } else {
        point_color.g = 1;
        point_color.a = 1;
      }
      room_marker.colors.push_back(point_color);
    }

    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(room_marker);
    room_centers_pub->publish(markers);

  }

 private:
 rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
      skeleton_graph_sub ;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      corrected_odom_sub;
  // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
  //     cluster_cloud_sub;
  // rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
  //     cluster_cloud_marker_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      is_room_completed_sub;
  rclcpp::Subscription<situational_graphs_msgs::msg::PlanesData>::SharedPtr
      map_planes_sub;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_cloud_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_clouds_pub;
  rclcpp::Publisher<situational_graphs_msgs::msg::RoomsData>::SharedPtr room_data_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr room_centers_pub;

  /* private variables */
 private:
  rclcpp::TimerBase::SharedPtr room_detection_timer;
  std::mutex map_plane_mutex, cloud_mutex;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  geometry_msgs::msg::TransformStamped tf_local2global;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster; //stacked
  visualization_msgs::msg::MarkerArray curr_cloud_marker;
  std::vector<situational_graphs_msgs::msg::PlaneData> x_vert_plane_vec,
      y_vert_plane_vec;
  std::deque<std::vector<situational_graphs_msgs::msg::PlaneData>> x_vert_plane_queue,
      y_vert_plane_queue;

  std::unique_ptr<SubRoomAnalyzer> room_analyzer;
  std::string map_frame_id = "map";
  bool save_timings;
  std::ofstream time_recorder;
  std::string output_file;
  int roomseg_timer;

  int cloud_passed;
  std::deque<geometry_msgs::msg::PoseStamped> odom_buf;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud_cluster; //before room seg
  double prev_x_width, prev_y_width;
};

}  // namespace s_graphs

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<s_graphs::SubRoomSegmentationNode>());
  rclcpp::shutdown();
  return 0;
}
