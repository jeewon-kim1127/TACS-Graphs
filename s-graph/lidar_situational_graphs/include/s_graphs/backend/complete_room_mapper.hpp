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

#ifndef COMPLETE_ROOM_MAPPER_HPP
#define COMPLETE_ROOM_MAPPER_HPP

#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <math.h>

#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <g2o/edge_infinite_room_plane.hpp>
#include <g2o/edge_plane.hpp>
#include <g2o/edge_room.hpp>
#include <g2o/vertex_infinite_room.hpp>
#include <g2o/vertex_room.hpp>
#include <s_graphs/backend/graph_slam.hpp>
#include <s_graphs/common/infinite_rooms.hpp>
#include <s_graphs/common/plane_utils.hpp>
#include <s_graphs/common/planes.hpp>
#include <s_graphs/common/rooms.hpp>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "situational_graphs_msgs/msg/rooms_data.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <s_graphs/common/complete_rooms.hpp>
#include "situational_graphs_msgs/msg/complete_room_data.hpp"
#include "situational_graphs_msgs/msg/complete_rooms_data.hpp"

namespace s_graphs {

class CompleteRoomMapper : public MapperUtils {
  typedef pcl::PointXYZRGBNormal PointNormal;

 public:
  /**
   * @brief Constructor of class CompleteRoomMapper.
   *
   * @param private_nh
   */
  CompleteRoomMapper(const rclcpp::Node::SharedPtr node);
  ~CompleteRoomMapper();

 private:
  rclcpp::Node::SharedPtr node_obj;

 public:
 
  bool lookup_rooms(
      std::shared_ptr<GraphSLAM>& graph_slam,
      const situational_graphs_msgs::msg::CompleteRoomData room_data,
      const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
      const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
      std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes,
      std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes,
      std::unordered_map<int, CompleteRooms>& rooms_vec,
      int& room_id);

  /**
   * @brief
   *
   * @param room_pose
   * @param plane
   * @return
   */
  double room_measurement(const Eigen::Vector2d& room_pose,
                          const Eigen::Vector4d& plane);

 private:
  /**
   * @brief Creates the room vertex and adds edges between the vertex and
   * detected planes
   *
   * @param graph_slam
   * @param x_room_pair_vec
   * @param y_room_pair_vec
   * @param x_vert_planes
   * @param y_vert_planes
   * @param dupl_x_vert_planes
   * @param dupl_y_vert_planes
   * @param rooms_vec
   * @param cluster_array
   */
  bool factor_rooms(
      std::shared_ptr<GraphSLAM>& graph_slam,
      std::vector<plane_data_list> x_room_pair_vec,
      std::vector<plane_data_list> y_room_pair_vec,
      const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
      const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
      std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes,
      std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes,
      std::unordered_map<int, CompleteRooms>& rooms_vec,
      int& room_id,
      const Eigen::Isometry3d& room_center,
      const visualization_msgs::msg::MarkerArray& cluster_array);

  /**
   * @brief
   *
   * @param room_pose
   * @param rooms_vec
   * @param x_vert_planes
   * @param y_vert_planes
   * @param x_plane1
   * @param x_plane2
   * @param y_plane1
   * @param y_plane2
   * @param detected_mapped_plane_pairs
   * @return
   */
  int associate_rooms(const Eigen::Isometry3d& room_center,
                      const std::unordered_map<int, CompleteRooms>& rooms_vec,
                      const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
                      const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
                      const VerticalPlanes& x_plane1,
                      const VerticalPlanes& x_plane2,
                      const VerticalPlanes& y_plane1,
                      const VerticalPlanes& y_plane2,
                      std::vector<std::pair<VerticalPlanes, VerticalPlanes>>&
                          detected_mapped_plane_pairs);

  /**
   * @brief
   *
   * @param plane_edges
   * @param room_node
   * @return Success or failure
   */
  bool check_room_ids(const std::set<g2o::HyperGraph::Edge*>& plane_edges,
                      const g2o::VertexRoom* room_node);
  bool check_plane_ids(const std::set<g2o::HyperGraph::Edge*>& plane_edges,
                       const g2o::VertexPlane* plane_node);
  /**
   * @brief Map a new room from mapped infinite_room planes
   *
   * @param graph_slam
   * @param det_room_data
   * @param matched_x_infinite_room
   * @param matched_y_infinite_room
   * @param rooms_vec
   * @param x_vert_planes
   * @param y_vert_planes
   * @param x_plane1
   * @param x_plane2
   * @param y_plane1
   * @param y_plane2
   */
  void map_room_from_existing_complete_rooms(
      std::shared_ptr<GraphSLAM>& graph_slam,
      const situational_graphs_msgs::msg::CompleteRoomData& det_room_data,
      const s_graphs::CompleteRooms& matched_complete_room,
      const Eigen::Isometry3d& room_center,
      std::unordered_map<int, Rooms>& rooms_vec,
      const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
      const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
      const VerticalPlanes& x_plane1,
      const VerticalPlanes& x_plane2,
      const VerticalPlanes& y_plane1,
      const VerticalPlanes& y_plane2);
  /**
   * @brief remove the infinite_room overlapped by a room
   *
   */
  void remove_mapped_complete_room(
      std::shared_ptr<GraphSLAM>& graph_slam,
      const s_graphs::CompleteRooms& matched_complete_room,
      std::unordered_map<int, CompleteRooms>& complete_rooms);

 private:
  double room_information;
  double room_dist_threshold;
  bool use_parallel_plane_constraint, use_perpendicular_plane_constraint;
  double dupl_plane_matching_information;
};

}  // namespace s_graphs

#endif  // COMPLETE_ROOM_MAPPER_HPP
