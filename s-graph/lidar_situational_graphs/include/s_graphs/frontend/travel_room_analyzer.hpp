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

#ifndef TRAVEL_ROOM_ANALYZER_HPP
#define TRAVEL_ROOM_ANALYZER_HPP

#include <math.h>
#include <pcl/common/angles.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <boost/format.hpp>
#include <cmath>
#include <iostream>
#include <s_graphs/common/plane_utils.hpp>
#include <string>
#include <deque>
#include <memory>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "situational_graphs_msgs/msg/plane_data.hpp"
#include "situational_graphs_msgs/msg/planes_data.hpp"
#include "situational_graphs_msgs/msg/rooms_data.hpp"
#include "situational_graphs_msgs/msg/complete_room_data.hpp"
#include "situational_graphs_msgs/msg/complete_rooms_data.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <s_graphs/frontend/room_analyzer.hpp>

#include "tf2/convert.h"
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std;

namespace s_graphs {

// struct room_analyzer_params {
//   long int vertex_neigh_thres;
// };

// struct RoomInfo {
//  public:
//   const std::vector<situational_graphs_msgs::msg::PlaneData>& current_x_vert_planes;
//   const std::vector<situational_graphs_msgs::msg::PlaneData>& current_y_vert_planes;
//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster;
// };

// struct RoomPlanes {
//  public:
//   situational_graphs_msgs::msg::PlaneData& x_plane1;
//   situational_graphs_msgs::msg::PlaneData& x_plane2;
//   situational_graphs_msgs::msg::PlaneData& y_plane1;
//   situational_graphs_msgs::msg::PlaneData& y_plane2;
//   bool found_x1_plane;
//   bool found_x2_plane;
//   bool found_y1_plane;
//   bool found_y2_plane;
// };

/**
 * @brief Class that provides tools for different analysis over open space
 * clusters to generate rooms
 */
class TravelRoomAnalyzer {
 public:
  /**
   * @brief Constructor of class TravelRoomAnalyzer
   *
   * @param private_nh
   * @param plane_utils_ptr
   */
  TravelRoomAnalyzer(room_analyzer_params params, rclcpp::Clock::SharedPtr clock);
  ~TravelRoomAnalyzer();

  void set_odom_buf(const std::deque<geometry_msgs::msg::PoseStamped>& odom_buf_in){
    if (odom_buf_in.empty()) return;
    odom_buf.clear();
    odom_buf = odom_buf_in;
    return;
  }

  void set_previous_global_cloud_hull(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_hull){
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_hull(new pcl::PointCloud<pcl::PointXYZRGB>());
    // for (auto& pt : cloud_hull->points)
    // tmp_hull->points.push_back(pt);
    // previous_global_cloud_hull = tmp_hull;
    previous_global_cloud_hull = cloud_hull;
    return;
  }

  visualization_msgs::msg::Marker get_cur_pose(){
    return curr_position_marker;
  }

  bool check_completion(){
    return is_completed;
  }
  bool check_inside_room(int& graph_idx){
    if (farthest_room_graph_idx == graph_idx) return false;
    return true;
  }
  /**
   * @brief
   *
   * @param skeleton_graph_msg
   */
  void analyze_skeleton_graph(
      const visualization_msgs::msg::MarkerArray::SharedPtr& skeleton_graph_msg);


  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr extract_current_cloud_cluster() {
  //   return local_cloud_cluster;
  // }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr extract_cloud_cluster() {
    return cloud_cluster;
  }

  visualization_msgs::msg::MarkerArray extract_marker_array_clusters() {
    return local_clusters_marker_array;
  }

  /**
   * @brief
   *
   * @param current_x_vert_planes
   * @param current_y_vert_planes
   * @param p_min
   * @param p_max
   * @param cloud_hull
   * @param x_plane1
   * @param x_plane2
   * @param y_plane1
   * @param y_plane2
   * @param found_x1_plane
   * @param found_x2_plane
   * @param found_y1_plane
   * @param found_y2_plane
   * @return
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr extract_room_planes(RoomInfo& room_info,
                                                             RoomPlanes& room_planes,
                                                             pcl::PointXY p_min,
                                                             pcl::PointXY p_max);

  /**
   * @brief
   *
   * @param skeleton_cloud
   * @param cloud_hull
   * @param area
   */
  void extract_convex_hull(pcl::PointCloud<pcl::PointXYZRGB>::Ptr skeleton_cloud,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_hull,
                           float& area);

  /**
   * @brief
   *
   * @param skeleton_cloud
   * @param p1
   * @param p2
   */
  void extract_cluster_endpoints(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& skeleton_cloud,
      pcl::PointXY& p1,
      pcl::PointXY& p2);

  /**
   * @brief
   *
   * @param skeleton_cloud
   * @param p1
   * @param p2
   * @return Success or Failure
   */
  bool extract_centroid_location(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& skeleton_cloud,
      const pcl::PointXY& p1,
      const pcl::PointXY& p2);

  /**
   * @brief
   *
   * @param skeleton_cloud
   * @param room_center
   * @return Success or Failure
   */
  bool extract_centroid_location(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& skeleton_cloud,
      const geometry_msgs::msg::Point& room_center);

  /**
   * @brief Compute the distance between 2 points, p1 and p2.
   *
   * @param p1
   * @param p2
   * @return Distance between point p1 and point p2.
   */
  geometry_msgs::msg::Point extract_room_length(const pcl::PointXY& p1,
                                                const pcl::PointXY& p2);

  /**
   * @brief Compute the center of a room.
   *
   * @param plane_type
   * @param p1
   * @param p2
   * @param plane1
   * @param plane2
   * @param cluster_center
   * @return The center point of the room.
   */
  geometry_msgs::msg::Point extract_infinite_room_center(
      int plane_type,
      pcl::PointXY p1,
      pcl::PointXY p2,
      situational_graphs_msgs::msg::PlaneData plane1,
      situational_graphs_msgs::msg::PlaneData plane2,
      Eigen::Vector2d& cluster_center);


  // bool perform_room_segmentation(
  //     RoomInfo& room_info,
  //     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster,
  //     std::vector<situational_graphs_msgs::msg::RoomData>& room_candidates_vec,
  //     const visualization_msgs::msg::MarkerArray& cloud_marker_array);

  bool perform_room_segmentation(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster,
    std::vector<situational_graphs_msgs::msg::RoomData>& rooms_vec,
    const std::vector<situational_graphs_msgs::msg::PlaneData>& current_x_vert_planes,
    const std::vector<situational_graphs_msgs::msg::PlaneData>& current_y_vert_planes,
    std::vector<situational_graphs_msgs::msg::CompleteRoomData>& room_candidate_vec) ;
  
  void downsample_cloud_data(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_hull);


// Find the axis-aligned bounding box of a point cloud
  void findBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_hull, float& min_x, float& min_y, float& max_x, float& max_y) {
    min_x = std::numeric_limits<float>::max();
    min_y = std::numeric_limits<float>::max();
    max_x = std::numeric_limits<float>::lowest();
    max_y = std::numeric_limits<float>::lowest();

    for (const auto& point : cloud_hull->points) {
        if (point.x < min_x) min_x = point.x;
        if (point.y < min_y) min_y = point.y;
        if (point.x > max_x) max_x = point.x;
        if (point.y > max_y) max_y = point.y;
    }
  }

  // Compute the smallest square that encloses the bounding box
  void computeSmallestEnclosingSquare(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& square_corners, float min_x, float min_y, float max_x, float max_y) {
    square_corners->clear();
    
    float x_side_length = abs(max_x - min_x)*0.5;
    float y_side_length = abs(max_y - min_y)*0.5;

    float center_x = (min_x + max_x)*0.5;
    float center_y = (min_y + max_y)*0.5;

    pcl::PointXYZRGB corner1, corner2, corner3, corner4;
    corner1.x = center_x - x_side_length;
    corner1.y = center_y - y_side_length;
    corner1.z = 0.0;

    corner2.x = center_x + x_side_length;
    corner2.y = center_y - y_side_length;
    corner2.z = 0.0;

    corner3.x = center_x + x_side_length;
    corner3.y = center_y + y_side_length;
    corner3.z = 0.0;

    corner4.x = center_x - x_side_length;
    corner4.y = center_y + y_side_length;
    corner4.z = 0.0;
    
    square_corners->points.push_back(corner1);
    square_corners->points.push_back(corner2);
    square_corners->points.push_back(corner3);
    square_corners->points.push_back(corner4);

    return ;
  }

 private:
  /**
   * @brief
   *
   * @param x_plane1_points
   * @param y_plane_points
   * @return Aligned or not aligned
   */
  bool is_x1_plane_aligned_w_y(
      const std::vector<geometry_msgs::msg::Vector3> x_plane1_points,
      const std::vector<geometry_msgs::msg::Vector3> y_plane_points);

  /**
   * @brief
   *
   * @param x_plane2_points
   * @param y_plane_point
   * @return Aligned or not aligned
   */
  bool is_x2_plane_aligned_w_y(
      const std::vector<geometry_msgs::msg::Vector3> x_plane2_points,
      const std::vector<geometry_msgs::msg::Vector3> y_plane_point);

  /**
   * @brief
   *
   * @param y_plane1_points
   * @param x_plane_points
   * @return Aligned or not aligned
   */
  bool is_y1_plane_aligned_w_x(
      const std::vector<geometry_msgs::msg::Vector3> y_plane1_points,
      const std::vector<geometry_msgs::msg::Vector3> x_plane_points);

  /**
   * @brief
   *
   * @param y_plane2_points
   * @param x_plane_points
   * @return Aligned or not aligned
   */
  bool is_y2_plane_aligned_w_x(
      const std::vector<geometry_msgs::msg::Vector3> y_plane2_points,
      const std::vector<geometry_msgs::msg::Vector3> x_plane_points);

 private:
  int vertex_neigh_thres;

  std::mutex skeleton_graph_mutex;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> local_cloud_clusters;
  visualization_msgs::msg::MarkerArray local_clusters_marker_array;
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_cloud_cluster;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster; //global pcl inside room
//   std::vector<std::pair<int, int>> local_subgraphs;
//   std::vector<std::pair<int, int>> subgraphs;

//   std::vector<pcl::PointXY> local_subgraph_centers;
  pcl::PointXYZRGB cloud_hull_center;
  int num_cloud_clusters;

  int disconnection;
  pcl::PointXYZRGB disconnection_start, disconnection_end;


 private:
  /**
   * @brief
   *
   * @param start_point
   * @param end_point
   * @param plane
   * @return
   */
  std::vector<float> find_plane_points(
      const pcl::PointXY& start_point,
      const pcl::PointXY& end_point,
      const situational_graphs_msgs::msg::PlaneData& plane);

  /**
   * @brief
   *
   * @param cloud_hull
   * @param plane
   * @param sub_cloud_cluster
   * @return
   */
  int find_plane_points(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_hull,
                        const situational_graphs_msgs::msg::PlaneData& plane,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr& sub_cloud_cluster);

  /**
   * @brief
   *
   * @param p1
   * @param p2
   * @return
   */
  pcl::PointXYZRGB compute_centroid(const pcl::PointXYZRGB& p1,
                                    const pcl::PointXYZRGB& p2);



Eigen::Affine3f transformToEigen(const geometry_msgs::msg::TransformStamped &transform)
{
    Eigen::Translation3f translation(transform.transform.translation.x,
                                     transform.transform.translation.y,
                                     transform.transform.translation.z);

    Eigen::Quaternionf rotation(transform.transform.rotation.w,
                                transform.transform.rotation.x,
                                transform.transform.rotation.y,
                                transform.transform.rotation.z);

    return translation * rotation;
}


  Eigen::Vector3d pointMsgToEigen(const geometry_msgs::msg::Point& point_msg) {
    return Eigen::Vector3d(point_msg.x, point_msg.y, point_msg.z);
  }
  Eigen::Quaterniond quaternionMsgToEigen(const geometry_msgs::msg::Quaternion& quat_msg) {
    return Eigen::Quaterniond(quat_msg.w, quat_msg.x, quat_msg.y, quat_msg.z);
  }
  void transformLocal2Global(const Eigen::Matrix4d &transform,
                          const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &local_cloud_in,
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr &global_cloud_out) {
    global_cloud_out->clear();            
    Eigen::Vector4d pt_in_eig, pt_out_eig;
    for (auto pt : local_cloud_in->points) {
      pt_in_eig << pt.x, pt.y, pt.z, 1;
      pt_out_eig = transform * pt_in_eig;
      pcl::PointXYZRGB pt_out;
      pt_out = pt;
      pt_out.x = pt_out_eig[0];
      pt_out.y = pt_out_eig[1];
      pt_out.z = pt_out_eig[2];
      global_cloud_out->push_back(pt_out);
    }
    return;
  };

//   room_analyzer_params params_;

  std::deque<geometry_msgs::msg::PoseStamped> odom_buf;
  geometry_msgs::msg::PoseStamped current_odom;
  visualization_msgs::msg::Marker curr_position_marker;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  geometry_msgs::msg::TransformStamped tf_local2global;

  int complete_room_id;
  bool is_completed;
  bool is_insid_room;
  int farthest_room_graph_idx;  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr previous_global_cloud_hull;                               
};
}  // namespace s_graphs

#endif  // TRAVEL_ROOM_ANALYZER_HPP
