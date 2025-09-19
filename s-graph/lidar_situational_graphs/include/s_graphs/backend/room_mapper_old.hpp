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

#ifndef ROOM_MAPPER_OLD_HPP
#define ROOM_MAPPER_OLD_HPP

#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <math.h>
#include <cmath>

#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <g2o/edge_infinite_room_plane.hpp>
#include <g2o/edge_plane.hpp>
#include <g2o/edge_plane_identity.hpp>
#include <g2o/edge_room.hpp>
#include <g2o/vertex_infinite_room.hpp>
#include <g2o/vertex_room.hpp>
#include <s_graphs/backend/room_mapper.hpp>
#include <s_graphs/backend/graph_slam.hpp>
#include <s_graphs/common/infinite_rooms.hpp>
#include <s_graphs/common/room_utils.hpp>
#include <s_graphs/common/plane_utils.hpp>
#include <s_graphs/common/planes.hpp>
#include <s_graphs/common/rooms.hpp>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "situational_graphs_msgs/msg/rooms_data.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
using namespace std;

namespace s_graphs {

/**
 * @brief
 */
// class OldMapperUtils {
//  public:
//   /**
//    * @brief Constructor of the class OldMapperUtils
//    *
//    * @param
//    * @return
//    */
//   OldMapperUtils() {}

//  public:
//   /**
//    * @brief
//    *
//    * @param plane_type
//    * @param p1
//    * @param p2
//    * @return
//    */
//   inline float point_difference(int plane_type, pcl::PointXY p1, pcl::PointXY p2) {
//     float point_diff = 0;

//     if (plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
//       p1.x = 0;
//       p2.x = 0;
//       point_diff = pcl::euclideanDistance(p1, p2);
//     }
//     if (plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
//       p1.y = 0;
//       p2.y = 0;
//       point_diff = pcl::euclideanDistance(p1, p2);
//     }

//     return point_diff;
//   }

//   /**
//    * @brief This method add parallel constraint between the planes of rooms or
//    * infinite_rooms
//    *
//    * @param graph_slam
//    * @param plane1_node
//    * @param plane2_node
//    */
//   void parallel_plane_constraint(std::shared_ptr<GraphSLAM>& graph_slam,
//                                  g2o::VertexPlane* plane1_node,
//                                  g2o::VertexPlane* plane2_node) {
//     Eigen::Matrix<double, 1, 1> information(0.1);
//     Eigen::Vector3d meas(0, 0, 0);

//     auto edge = graph_slam->add_plane_parallel_edge(
//         plane1_node, plane2_node, meas, information);
//     graph_slam->add_robust_kernel(edge, "Huber", 1.0);
//   }

//   /**
//    * @brief This method adds perpendicular constraint between the planes of
//    * rooms or infinite_rooms
//    *
//    * @param graph_slam
//    * @pram plane1_node
//    * @pram plane2_node
//    */
//   void perpendicular_plane_constraint(std::shared_ptr<GraphSLAM>& graph_slam,
//                                       g2o::VertexPlane* plane1_node,
//                                       g2o::VertexPlane* plane2_node) {
//     Eigen::Matrix<double, 1, 1> information(0.1);
//     Eigen::Vector3d meas(0, 0, 0);

//     auto edge = graph_slam->add_plane_perpendicular_edge(
//         plane1_node, plane2_node, meas, information);
//     graph_slam->add_robust_kernel(edge, "Huber", 1.0);
//   }

//   bool check_plane_ids(const std::set<g2o::HyperGraph::Edge*>& plane_edges,
//                        const g2o::VertexPlane* plane_node) {
//     for (auto edge_itr = plane_edges.begin(); edge_itr != plane_edges.end();
//          ++edge_itr) {
//       g2o::Edge2Planes* edge_2planes = dynamic_cast<g2o::Edge2Planes*>(*edge_itr);
//       if (edge_2planes) {
//         g2o::VertexPlane* found_plane1_node =
//             dynamic_cast<g2o::VertexPlane*>(edge_2planes->vertices()[0]);
//         g2o::VertexPlane* found_plane2_node =
//             dynamic_cast<g2o::VertexPlane*>(edge_2planes->vertices()[1]);

//         if (found_plane1_node->id() == plane_node->id() ||
//             found_plane2_node->id() == plane_node->id())
//           return true;
//       }
//     }

//     return false;
//   }

//   bool check_plane_identity_ids(const std::set<g2o::HyperGraph::Edge*>& plane_edges,
//                        const g2o::VertexPlane* plane_node) {
//     for (auto edge_itr = plane_edges.begin(); edge_itr != plane_edges.end();
//          ++edge_itr) {
//       g2o::EdgePlaneIdentity* edge_plane_identity = dynamic_cast<g2o::EdgePlaneIdentity*>(*edge_itr);
//       if (edge_plane_identity) {
//         g2o::VertexPlane* found_plane1_node =
//             dynamic_cast<g2o::VertexPlane*>(edge_plane_identity->vertices()[0]);
//         g2o::VertexPlane* found_plane2_node =
//             dynamic_cast<g2o::VertexPlane*>(edge_plane_identity->vertices()[1]);

//         if (found_plane1_node->id() == plane_node->id() ||
//             found_plane2_node->id() == plane_node->id())
//           return true;
//       }
//     }

//     return false;
//   }

//   bool extract_centroid_location(
//       const visualization_msgs::msg::MarkerArray& cloud_marker_array,
//       const geometry_msgs::msg::Point& room_center) {
//     float min_dist = 100;
//     // std::cout<<"cloud_marker_array "<<cloud_marker_array.markers.size()<<std::endl;
//     for (auto marker : cloud_marker_array.markers) {
//       // std::cout<<"marker.points.size() "<<marker.points.size()<<std::endl;
//       for (auto pt: marker.points) {
//         float dist = sqrt(pow(room_center.x - pt.x, 2) +
//                           pow(room_center.y - pt.y, 2));
//         if (dist < min_dist) {
//           min_dist = dist;
//           if (min_dist<=1.0) return true;
//         }
//       }
//     }
//     if (min_dist > 1.0) {
//       std::cout<<"extract_centroid_location "<<min_dist<<std::endl;
//       return false;
//     }
//     return true;
//   }

//   double calculateDistance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2) {
//     return std::sqrt(
//         std::pow(p1.x - p2.x, 2) + 
//         std::pow(p1.y - p2.y, 2)); //  std::pow(p1.z - p2.z, 2)
//   }
  
//   void mergeMarkerArrays(Rooms &room,
//     const visualization_msgs::msg::MarkerArray &array2, 
//     double min_distance = 0.05)
//   {
//     for (const auto &marker2 : array2.markers) {
//       room.cluster_array.markers.push_back(marker2);
//     }
//     // visualization_msgs::msg::MarkerArray array1 = room.cluster_array;
//     // for (const auto &marker2 : array2.markers) {
//     //     bool is_far_enough = true;

//     //     for (const auto &marker1 : room.cluster_array.markers) {
//     //         double distance = calculateDistance(marker1.pose.position, marker2.pose.position);
//     //         if (distance < min_distance) {
//     //             is_far_enough = false;  // 거리가 너무 가까우면 추가하지 않음
//     //             break;
//     //         }
//     //     }
//     //     if (is_far_enough) {
//     //         room.cluster_array.markers.push_back(marker2);
//     //     }
//     // }
//   }

//   void mergeMarkerArrays(InfiniteRooms &room, 
//     const std::string & room_type,
//     const visualization_msgs::msg::MarkerArray &array2, 
//     double min_distance = 0.05)
//   {
//     visualization_msgs::msg::MarkerArray array1 = room.cluster_array;
//     std::cout<<" room.cluster_array.markers "<< room.cluster_array.markers.size()<<
//             " array2.markers "<<array2.markers.size()<<std::endl;
//     // for (const auto &marker2 : array2.markers) {
//     //     bool is_far_enough = true;
//     //     for (const auto &marker1 : room.cluster_array.markers) {
//     //         double distance = calculateDistance(marker1.pose.position, marker2.pose.position);
//     //         // std::cout<<"distance "<<distance<<" min_distance "<<min_distance<<std::endl;
//     //         if (distance < min_distance) {
//     //             is_far_enough = false;  // 거리가 너무 가까우면 추가하지 않음
//     //             break;
//     //         }
//     //     }
//     //     if (is_far_enough) {
//     //         marker.push_back(marker2);
//     //     }
//     // }
//     // room.cluster_array.markers.push_back(marker);
//      for (const auto &marker2 : array2.markers) {
//         // bool outside = false;
//         // for (const auto &pt : marker2.points){
//         //     if (room_type=="x" && (pt.y<room.room_min || pt.y>room.room_max)){
//         //         outside = true;
//         //         break;
//         //     } else if (room_type=="y" && (pt.x<room.room_min || pt.x>room.room_max)){
//         //         outside = true;
//         //         break;
//         //     }
//         // }
//         // if (outside) 
//         room.cluster_array.markers.push_back(marker2);
//     }
//     // std::cout<<"after mergeMarkerArrays "<< room.cluster_array.markers.size()<<std::endl;
//   }
// //////////////////
//   void setRoomCenter(InfiniteRooms &room, const std::string room_type,
//                         const Eigen::Isometry3d &new_room_center)
//   {
//     Eigen::Isometry3d updated_room_center = Eigen::Isometry3d::Identity();
//     Eigen::Vector3d avg_translation = (new_room_center.translation() + room.node->estimate().translation()) / 2.0;
//     if (room_type=="x") {
//         avg_translation.y() = (room.room_max + room.room_min) / 2.0;
//         std::cout<<"setRoomCenter: before "<<room.node->estimate().translation().y()<<" -> updated "<<avg_translation.y() <<std::endl;
//     }
//     else if (room_type=="y") {
//         avg_translation.x() = (room.room_max + room.room_min) / 2.0;
//         std::cout<<"setRoomCenter: before "<<room.node->estimate().translation().x()<<" -> updated "<<avg_translation.x() <<std::endl;
//     }
//     updated_room_center.translation() = avg_translation;
//     updated_room_center.linear() = new_room_center.rotation();
//     room.node->setEstimate(updated_room_center);
//     return;
//   }

//   void setRoomCenter(Rooms &room,
//                         const Eigen::Isometry3d &new_room_center)
//   {
//     Eigen::Isometry3d updated_room_center = Eigen::Isometry3d::Identity();
//     Eigen::Vector3d avg_translation;
    
//     avg_translation.x() = (room.room_x_max + room.room_x_min) / 2.0;
//     avg_translation.y() = (room.room_y_max + room.room_y_min) / 2.0;
//     avg_translation.z() = new_room_center.translation().z();
//     std::cout<<"setRoomCenter: before ("<<room.node->estimate().translation().x()<<","<<room.node->estimate().translation().y()<<") -> updated ("
//               <<avg_translation.x()<<"," <<avg_translation.y() <<")"<<std::endl;

//     updated_room_center.translation() = avg_translation;
//     updated_room_center.linear() = new_room_center.rotation();
//     room.node->setEstimate(updated_room_center);
//     return;
//   }

//   ////////
//   void mergeKeyframes(
//             std::map<int, s_graphs::KeyFrame::Ptr> &keyframes1,
//             const std::map<int, s_graphs::KeyFrame::Ptr> &keyframes2,
//             const std::map<int, s_graphs::KeyFrame::Ptr> &keyframes){
//     bool same_kf = false;
//     for (const auto &kf2 : keyframes2) {
//         same_kf = false;
//         for (const auto &kf1 : keyframes1) {
//             if (kf1.second->id() == kf2.second->id()) {
//                 same_kf = true;
//                 break;
//             }
//         }
//         if (!same_kf) keyframes1.insert(kf2);
//     }
//   }
          
//   double calculateDistance(const double &x1,const double &y1,
//                             const double &x2,const double &y2,
//                             const double &roompart_length) {
//     return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));  
//     }  
// //   void mergePartRooms(
// //             std::map<int, std::tuple<double, double, std::map<int, s_graphs::KeyFrame::Ptr>>>& new_roompart,
// //             const std::map<int, std::tuple<double, double, std::map<int, s_graphs::KeyFrame::Ptr>>>& old_roompart,
// //             const double &roompart_length){
// //     bool same_kf = false;
// //     new_roompart.clear();
// //     for (auto &part : old_roompart) {
// //         new_roompart.insert({part.first,{std::get<0>(part.second),std::get<1>(part.second),{}}});
// //     }
// //     for (const auto &kf : new_keyframes) {
// //         same_kf = false;
// //         double kf_x = kf.second->odom.translation().x();
// //         double kf_y = kf.second->odom.translation().y();
// //         for (auto &part : old_roompart) {
// //             std::map<int, s_graphs::KeyFrame::Ptr> tmp_kfs;
// //             if (calculateDistance(std::get<0>(part.second),std::get<1>(part.second), kf_x, kf_y)
// //                         < roompart_length*0.5)
// //             {
// //                 for (const auto &kf1 : std::get<2>(part.second)) {
// //                     if (kf1.second->id() == kf.second->id()) {
// //                         same_kf = true;
// //                         break;
// //                     }
// //                 }
// //                 if (!same_kf){
// //                     std::get<2>(new_roompart[part.first]).insert(kf1);
// //                     break;
// //                 }
// //             }
// //         }
// //     }
// //   }


// template <typename KeyFramePtrVec> //std::map<int, s_graphs::KeyFrame::Ptr>
// void update_old_roompart_keyframes(
//             const KeyFramePtrVec& keyframes,
//             const std::string& room_type,
//             const double& room_start, const double& room_end,
//             const double& roompart_length,
//             std::map<int, std::tuple<double, double, KeyFramePtrVec>>& old_roompart) {
//   int part_num = 0; 
//   double mid_x = 0; double mid_y = 0;
//   old_roompart.clear();
  
//   double tmp=room_start;
//   while (true){
//     // std::cout<<"tmp "<<tmp<<" "<<room_start<<" "<<room_end<<std::endl;
//     if (tmp >= room_end) break;
//     if (room_type=="x") mid_y = tmp+0.5*roompart_length;
//     if (room_type=="y") mid_x = tmp+0.5*roompart_length;
//     old_roompart.insert({part_num, {mid_x, mid_y, {}}});
//     part_num++;
//     tmp += roompart_length;
//   }
//   for (auto& kf : keyframes) {
//     double kf_tmp;
//     if (room_type=="x") kf_tmp = kf.second->odom.translation().y();
//     else kf_tmp = kf.second->odom.translation().x();
//     if (kf_tmp<room_start) continue;
    
//     int part_num_tmp = (int)((kf_tmp-room_start)/roompart_length);
//     std::get<2>(old_roompart[part_num_tmp]).insert(kf);
//   }
//   return ;
// }

// };

/**
 * @brief Class that provides tools for different analysis over open space
 * clusters to generate rooms
 */
class OldInfiniteRoomMapper : public MapperUtils {
  typedef pcl::PointXYZRGBNormal PointNormal;

 public:
  /**
   * @brief Constructor of the class OldInfiniteRoomMapper
   *
   * @param private_nh
   * @return
   */
  OldInfiniteRoomMapper(const rclcpp::Node::SharedPtr node);
  ~OldInfiniteRoomMapper();

 private:
  rclcpp::Node::SharedPtr node_obj;

 public:
  /**
   * @brief
   *
   * @param graph_slam
   * @param plane_type
   * @param room_data
   * @param x_vert_planes
   * @param y_vert_planes
   * @param dupl_x_vert_planes
   * @param dupl_y_vert_planes
   * @param x_infinite_rooms
   * @param y_infinite_rooms
   * @param rooms_vec
   * @return bool
   */
  bool lookup_infinite_rooms(
      std::shared_ptr<GraphSLAM>& graph_slam,
      const int& plane_type,
      const situational_graphs_msgs::msg::RoomData room_data,
      const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
      const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
      const std::map<int, s_graphs::KeyFrame::Ptr> keyframes,
      std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes,
      std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes,
      std::unordered_map<int, InfiniteRooms>& x_infinite_rooms,
      std::unordered_map<int, InfiniteRooms>& y_infinite_rooms,
      std::unordered_map<int, Rooms>& rooms_vec,
      int& room_id,
      bool& matched);
  
  bool lookup_infinite_rooms(
    std::shared_ptr<GraphSLAM>& graph_slam,
    const int& plane_type,
    const situational_graphs_msgs::msg::RoomData room_data,
    const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes,
    std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes,
    std::unordered_map<int, InfiniteRooms>& x_infinite_rooms,
    std::unordered_map<int, InfiniteRooms>& y_infinite_rooms,
    std::unordered_map<int, Rooms>& rooms_vec,
    int& room_id,
    bool& matched);

 private:
  /**
   * @brief Creates the infinite_room vertex and adds edges between the vertex
   * the detected planes
   *
   * @param graph_slam
   * @param plane_type
   * @param corr_plane1_pair
   * @param corr_plane2_pair
   * @param x_vert_planes
   * @param y_vert_planes
   * @param dupl_x_vert_planes
   * @param dupl_y_vert_planes
   * @param x_infinite_rooms
   * @param y_infinite_rooms
   */
  bool factor_infinite_rooms(
      std::shared_ptr<GraphSLAM>& graph_slam,
      const int plane_type,
      const plane_data_list& corr_plane1_pair,
      const plane_data_list& corr_plane2_pair,
      const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
      const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
      std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes,
      std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes,
      std::unordered_map<int, InfiniteRooms>& x_infinite_rooms,
      std::unordered_map<int, InfiniteRooms>& y_infinite_rooms,
      std::unordered_map<int, Rooms>& rooms_vec,
      int& room_id,
      const Eigen::Isometry3d& room_center,
      const geometry_msgs::msg::Point &room_length,
      const Eigen::Isometry3d& cluster_center,
      const visualization_msgs::msg::MarkerArray& cluster_array,
    const std::map<int, s_graphs::KeyFrame::Ptr> keyframes,
      bool& matched);

  bool factor_infinite_rooms(
      std::shared_ptr<GraphSLAM>& graph_slam,
      const int plane_type,
      const plane_data_list& corr_plane1_pair,
      const plane_data_list& corr_plane2_pair,
      const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
      const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
      std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes,
      std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes,
      std::unordered_map<int, InfiniteRooms>& x_infinite_rooms,
      std::unordered_map<int, InfiniteRooms>& y_infinite_rooms,
      int& room_id,
      const Eigen::Isometry3d& room_center,
      const geometry_msgs::msg::Point &room_length,
      const Eigen::Isometry3d& cluster_center,
      const visualization_msgs::msg::MarkerArray& cluster_array,
      bool& matched);

  void mergeMarkerArrays(InfiniteRooms &room, 
    const std::string & room_type,
    const visualization_msgs::msg::MarkerArray &array2)
  {
    visualization_msgs::msg::MarkerArray array1 = room.cluster_array;
     for (const auto &marker2 : array2.markers) {
        room.cluster_array.markers.push_back(marker2);
    }
  }

  void setRoomCenter(InfiniteRooms &room, const std::string room_type,
                        const Eigen::Isometry3d &new_room_center)
  {
    Eigen::Isometry3d updated_room_center = Eigen::Isometry3d::Identity();
    Eigen::Vector3d avg_translation = (new_room_center.translation() + room.node->estimate().translation()) / 2.0;
    if (room_type=="x") {
        avg_translation.y() = (room.room_max + room.room_min) / 2.0;
        std::cout<<"setRoomCenter: before "<<room.node->estimate().translation().y()<<" -> updated "<<avg_translation.y() <<std::endl;
    }
    else if (room_type=="y") {
        avg_translation.x() = (room.room_max + room.room_min) / 2.0;
        std::cout<<"setRoomCenter: before "<<room.node->estimate().translation().x()<<" -> updated "<<avg_translation.x() <<std::endl;
    }
    updated_room_center.translation() = avg_translation;
    updated_room_center.linear() = new_room_center.rotation();
    room.node->setEstimate(updated_room_center);
    return;
  }

int associate_rooms(
    const int& plane_type,
    const Eigen::Isometry3d& room_center,
    const geometry_msgs::msg::Point &room_length,
    const visualization_msgs::msg::MarkerArray& cluster_array,
    const VerticalPlanes& plane1,
    const VerticalPlanes& plane2,
    const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    const std::unordered_map<int, Rooms>& rooms_vec,
    std::vector<std::pair<VerticalPlanes, VerticalPlanes>>&
        detected_mapped_plane_pairs) ;

  /**
   * @brief
   *
   * @param plane_type
   * @param corr_pose
   * @param plane1
   * @param plane2
   * @param x_vert_planes
   * @param y_vert_planes
   * @param x_infinite_rooms
   * @param y_infinite_rooms
   * @param detected_mapped_plane_pairs
   * @return
   */
  int associate_infinite_rooms(
      const int& plane_type,
      const Eigen::Isometry3d& room_center,
      const VerticalPlanes& plane1,
      const VerticalPlanes& plane2,
      const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
      const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
      const std::unordered_map<int, InfiniteRooms>& x_infinite_rooms,
      const std::unordered_map<int, InfiniteRooms>& y_infinite_rooms,
      std::vector<std::pair<VerticalPlanes, VerticalPlanes>>&
          detected_mapped_plane_pairs);
  
  int associate_infinite_rooms(
      const int& plane_type,
      const Eigen::Isometry3d& room_center,
      const geometry_msgs::msg::Point &room_length,
    const visualization_msgs::msg::MarkerArray& cluster_array,
      const VerticalPlanes& plane1,
      const VerticalPlanes& plane2,
      const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
      const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
      const std::unordered_map<int, InfiniteRooms>& x_infinite_rooms,
      const std::unordered_map<int, InfiniteRooms>& y_infinite_rooms,
      std::vector<std::pair<VerticalPlanes, VerticalPlanes>>&
          detected_mapped_plane_pairs);

  /**
   * @brief
   *
   * @param plane_type
   * @param plane_edges
   * @param corr_node
   * @return Success or failure
   */
  bool check_infinite_room_ids(const int plane_type,
                               const std::set<g2o::HyperGraph::Edge*>& plane_edges,
                               const g2o::VertexRoom* corr_node);
   
//////////////////
          
  float euclideanDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) {
      return std::sqrt(std::pow(p2.x - p1.x, 2) + 
                      std::pow(p2.y - p1.y, 2));
  }

  geometry_msgs::msg::Point plane_length(const visualization_msgs::msg::MarkerArray& marker_array) {
      // Initialize minimum and maximum points
      geometry_msgs::msg::Point pmin;
      geometry_msgs::msg::Point pmax;
      geometry_msgs::msg::Point length;
      
      // Set initial values to extreme opposites
      pmin.x = pmin.y = std::numeric_limits<float>::max();
      pmax.x = pmax.y = std::numeric_limits<float>::lowest();

      // Iterate through each Marker in the MarkerArray
      for (const auto& marker : marker_array.markers) {
          // Check if the marker has points
          if (marker.points.empty()) {
              continue;
          }

          // Iterate through the points in the marker
          for (const auto& point : marker.points) {
              // Update minimum and maximum points
              if (point.x < pmin.x)
                  pmin.x = point.x;
              else if (point.x > pmax.x)
                  pmax.x = point.x;
              if (point.y < pmin.y ) 
                  pmin.y = point.y;
              else if (point.y > pmax.y ) 
                  pmax.y = point.y ;
          }
      }
      length.x = abs(pmax.x-pmin.x);
      length.y = abs(pmax.y-pmin.y);
      return length;
  }

 private:
  /**
   * @brief
   *
   * @param graph_slam
   * @param plane1_node
   * @param plane2_node
   */
  void parallel_plane_constraint(std::shared_ptr<GraphSLAM>& graph_slam,
                                 g2o::VertexPlane* plane1_node,
                                 g2o::VertexPlane* plane2_node);
                                 

 private:
  double infinite_room_information;
  double infinite_room_dist_threshold;
  bool use_parallel_plane_constraint, use_perpendicular_plane_constraint;
  bool use_old_ablation;
  double dupl_plane_matching_information;
  int keyframe_window_size;
  double roompart_length;
};

/**
 * @brief
 */
class OldFiniteRoomMapper : public MapperUtils {
  typedef pcl::PointXYZRGBNormal PointNormal;

 public:
  /**
   * @brief Constructor of class OldFiniteRoomMapper.
   *
   * @param private_nh
   */
  OldFiniteRoomMapper(const rclcpp::Node::SharedPtr node);
  ~OldFiniteRoomMapper();

 private:
  rclcpp::Node::SharedPtr node_obj;

 public:
  /**
   * @brief
   *
   * @param graph_slam
   * @param room_data
   * @param x_vert_planes
   * @param y_vert_planes
   * @param dupl_x_vert_planes
   * @param dupl_y_vert_planes
   * @param x_infinite_rooms
   * @param y_infinite_rooms
   * @param rooms_vec
   */
  bool lookup_rooms(
      std::shared_ptr<GraphSLAM>& graph_slam,
      const situational_graphs_msgs::msg::RoomData room_data,
      const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
      const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
      const std::map<int, s_graphs::KeyFrame::Ptr>& keyframes,
      std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes,
      std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes,
      std::unordered_map<int, InfiniteRooms>& x_infinite_rooms,
      std::unordered_map<int, InfiniteRooms>& y_infinite_rooms,
      std::unordered_map<int, Rooms>& rooms_vec,
      int& room_id,
      bool& matched);

  bool lookup_rooms(
    std::shared_ptr<GraphSLAM>& graph_slam,
    const situational_graphs_msgs::msg::RoomData room_data,
    const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes,
    std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes,
    std::unordered_map<int, InfiniteRooms>& x_infinite_rooms,
    std::unordered_map<int, InfiniteRooms>& y_infinite_rooms,
    std::unordered_map<int, Rooms>& rooms_vec,
    int& room_id,
    bool& matched);
  /**
   * @brief
   *
   * @param plane_type
   * @param room_pose
   * @param plane
   * @return
   */
  double room_measurement(const int& plane_type,
                          const Eigen::Vector2d& room_pose,
                          const Eigen::Vector4d& plane);


  geometry_msgs::msg::Point plane_length(const visualization_msgs::msg::MarkerArray& marker_array) {
      // Initialize minimum and maximum points
      geometry_msgs::msg::Point pmin;
      geometry_msgs::msg::Point pmax;
      geometry_msgs::msg::Point length;
      
      // Set initial values to extreme opposites
      pmin.x = pmin.y = std::numeric_limits<float>::max();
      pmax.x = pmax.y = std::numeric_limits<float>::lowest();

      // Iterate through each Marker in the MarkerArray
      for (const auto& marker : marker_array.markers) {
          // Check if the marker has points
          if (marker.points.empty()) {
              continue;
          }

          // Iterate through the points in the marker
          for (const auto& point : marker.points) {
              // Update minimum and maximum points
              if (point.x < pmin.x)
                  pmin.x = point.x;
              else if (point.x > pmax.x)
                  pmax.x = point.x;
              if (point.y < pmin.y ) 
                  pmin.y = point.y;
              else if (point.y > pmax.y ) 
                  pmax.y = point.y ;
          }
      }
      length.x = abs(pmax.x-pmin.x);
      length.y = abs(pmax.y-pmin.y);
      return length;
  }


 private:

  void mergeMarkerArrays(Rooms &room,
    const visualization_msgs::msg::MarkerArray &array2)
  {
    for (const auto &marker2 : array2.markers) {
      room.cluster_array.markers.push_back(marker2);
    }
  };

  void setRoomCenter(Rooms &room,
                        const Eigen::Isometry3d &new_room_center)
  {
    Eigen::Isometry3d updated_room_center = Eigen::Isometry3d::Identity();
    Eigen::Vector3d avg_translation = (new_room_center.translation() + room.node->estimate().translation()) / 2.0;
    avg_translation.x() = (room.room_x_max + room.room_x_min) / 2.0;
    avg_translation.y() = (room.room_y_max + room.room_y_min) / 2.0;
    std::cout<<"setRoomCenter: before ("<<room.node->estimate().translation().x()<<","<<room.node->estimate().translation().y()<<") -> updated ("
              <<avg_translation.x()<<"," <<avg_translation.y() <<")"<<std::endl;

    updated_room_center.translation() = avg_translation;
    updated_room_center.linear() = new_room_center.rotation();
    room.node->setEstimate(updated_room_center);
    return;
  };

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
    std::unordered_map<int, Rooms>& rooms_vec,
    int& room_id,
    const Eigen::Isometry3d& room_center,
    const geometry_msgs::msg::Point &room_length,
    const visualization_msgs::msg::MarkerArray& cluster_array,
    bool& matched) ;

  bool factor_rooms(
      std::shared_ptr<GraphSLAM>& graph_slam,
      std::vector<plane_data_list> x_room_pair_vec,
      std::vector<plane_data_list> y_room_pair_vec,
      const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
      const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
      const std::map<int, s_graphs::KeyFrame::Ptr>& keyframes,
      std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes,
      std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes,
      std::unordered_map<int, Rooms>& rooms_vec,
      int& room_id,
      const Eigen::Isometry3d& room_center,
        const geometry_msgs::msg::Point &room_length,
        const visualization_msgs::msg::MarkerArray& cluster_array,
      bool& matched);

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

    //   int associate_rooms(
    //     const Eigen::Isometry3d& room_center,
    //     const std::unordered_map<int, Rooms>& rooms_vec,
    //     const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    //     const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    //     const VerticalPlanes& x_plane1,
    //     const VerticalPlanes& x_plane2,
    //     const VerticalPlanes& y_plane1,
    //     const VerticalPlanes& y_plane2,
    //     std::vector<std::pair<VerticalPlanes, VerticalPlanes>>&
    //         detected_mapped_plane_pairs);

  int associate_rooms(const Eigen::Isometry3d& room_center,
                        const geometry_msgs::msg::Point &room_length,
                      const std::unordered_map<int, Rooms>& rooms_vec,
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
   * @param plane_type
   * @param plane_edges
   * @param room_node
   * @return Success or failure
   */
  bool check_room_ids(const int plane_type,
                      const std::set<g2o::HyperGraph::Edge*>& plane_edges,
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
  void map_room_from_existing_infinite_rooms(
      std::shared_ptr<GraphSLAM>& graph_slam,
      const situational_graphs_msgs::msg::RoomData& det_room_data,
      const s_graphs::InfiniteRooms& matched_x_infinite_room,
      const s_graphs::InfiniteRooms& matched_y_infinite_room,
      const Eigen::Isometry3d& room_center,
      std::unordered_map<int, Rooms>& rooms_vec,
      const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
      const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    //   const std::map<int, s_graphs::KeyFrame::Ptr>& keyframes,
      const VerticalPlanes& x_plane1,
      const VerticalPlanes& x_plane2,
      const VerticalPlanes& y_plane1,
      const VerticalPlanes& y_plane2);

  /**
   * @brief map a new room from mapped x infinite_room planes
   *
   * @param graph_slam
   * @param det_room_data
   * @param matched_x_infinite_room
   * @param rooms_vec
   * @param x_vert_planes
   * @param y_vert_planes
   * @param x_plane1
   * @param x_plane2
   * @param y_plane1
   * @param y_plane2
   */
  void map_room_from_existing_x_infinite_room(
      std::shared_ptr<GraphSLAM>& graph_slam,
      const situational_graphs_msgs::msg::RoomData& det_room_data,
      const s_graphs::InfiniteRooms& matched_x_infinite_room,
      const Eigen::Isometry3d& room_center,
      std::unordered_map<int, Rooms>& rooms_vec,
      const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
      const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    //   const std::map<int, s_graphs::KeyFrame::Ptr>& keyframes,
      const VerticalPlanes& x_plane1,
      const VerticalPlanes& x_plane2,
      const VerticalPlanes& y_plane1,
      const VerticalPlanes& y_plane2);

  /**
   * @brief map a new room from mapped y infinite_room planes
   *
   * @param graph_slam
   * @param det_room_data
   * @param matched_y_infinite_room
   * @param rooms_vec
   * @param x_vert_planes
   * @param y_vert_planes
   * @param x_plane1
   * @param x_plane2
   * @param y_plane1
   * @param y_plane2
   */
  void map_room_from_existing_y_infinite_room(
      std::shared_ptr<GraphSLAM>& graph_slam,
      const situational_graphs_msgs::msg::RoomData& det_room_data,
      const s_graphs::InfiniteRooms& matched_y_infinite_room,
      const Eigen::Isometry3d& room_center,
      std::unordered_map<int, Rooms>& rooms_vec,
      const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
      const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    //   const std::map<int, s_graphs::KeyFrame::Ptr>& keyframes,
      const VerticalPlanes& x_plane1,
      const VerticalPlanes& x_plane2,
      const VerticalPlanes& y_plane1,
      const VerticalPlanes& y_plane2);

  /**
   * @brief remove the infinite_room overlapped by a room
   *
   */
  void remove_mapped_infinite_room(
      const int plane_type,
      std::shared_ptr<GraphSLAM>& graph_slam,
      s_graphs::InfiniteRooms matched_infinite_room,
      std::unordered_map<int, InfiniteRooms>& x_infinite_rooms,
      std::unordered_map<int, InfiniteRooms>& y_infinite_rooms);

 private:
  double room_information;
  double room_dist_threshold;
  bool use_parallel_plane_constraint, use_perpendicular_plane_constraint;
  bool use_old_ablation;

  double dupl_plane_matching_information;
  int keyframe_window_size;
  double roompart_length;
};

}  // namespace s_graphs

#endif  // ROOM_MAPPER_OLD_HPP