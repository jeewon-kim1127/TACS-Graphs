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
#include <s_graphs/frontend/travel_room_analyzer.hpp>
#include <s_graphs/frontend/room_analyzer.hpp>
#include <string>
#include <deque>

#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "rclcpp/rclcpp.hpp"
#include "situational_graphs_msgs/msg/rooms_data.hpp"
#include "situational_graphs_msgs/msg/room_data.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/bool.hpp"


namespace s_graphs {

class TravelRoomSegmentationNode : public rclcpp::Node {
 public:
  typedef pcl::PointXYZRGBNormal PointT;

  TravelRoomSegmentationNode() : Node("travel_room_segmentation_node") {
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
    this->declare_parameter("vertex_neigh_thres", 2);
    this->declare_parameter("save_timings", false);

    room_analyzer_params params{
        this->get_parameter("vertex_neigh_thres").get_parameter_value().get<int>()};
    save_timings =
        this->get_parameter("save_timings").get_parameter_value().get<bool>();
    
    room_analyzer.reset(new TravelRoomAnalyzer(params, this->get_clock()));
    cloud_cluster = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

    if (save_timings) {
      time_recorder.open("/tmp/room_seg_computation_time.txt");
      time_recorder << "#time \n";
      time_recorder.close();
    }
  }

  void init_ros() {
    skeleton_graph_sub =
        this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "travelgseg/edges",
            1,
            std::bind(&TravelRoomSegmentationNode::local_traversable_edges_callback,
                      this,
                      std::placeholders::_1));     //get local graphs
    corrected_odom_sub = 
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "s_graphs/odom_pose_corrected", 100, 
            std::bind(&TravelRoomSegmentationNode::corrected_odom_callback,
                      this,
                       std::placeholders::_1));
    // map_planes_sub =
    //     this->create_subscription<situational_graphs_msgs::msg::PlanesData>(
    //         "s_graphs/map_planes",
    //         100, [this](const situational_graphs_msgs::msg::PlanesData::SharedPtr msg) {
    //           this->map_planes_callback(msg);
    //       });   

    // rooms_data_sub = 
    //     this->create_subscription<situational_graphs_msgs::msg::RoomsData>(
    //         "room_segmentation/room_data", 100, 
    //         std::bind(&TravelRoomSegmentationNode::rooms_data_callback,
    //                   this,
    //                    std::placeholders::_1));

    rooms_data_sub = 
        this->create_subscription<situational_graphs_msgs::msg::RoomsData>(
            "room_segmentation/room_data", 100, 
            [this](const situational_graphs_msgs::msg::RoomsData::SharedPtr msg) {
              this->rooms_data_callback(msg);}
              );
    curr_pose_pub = this->create_publisher<visualization_msgs::msg::Marker>(
        "complete_room_segmentation/cur_pose", 1); 
    convex_hull_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "complete_room_segmentation/convex_hull_marker", 1);        ///////
    is_room_completed_pub = this->create_publisher<std_msgs::msg::Bool>(
        "complete_room_segmentation/is_room_completed", 1);
    current_cluster_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "complete_room_segmentation/current_cluster_cloud", 1);
    current_cloud_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "complete_room_segmentation/current_cloud_marker", 1);     
    complete_room_data_pub = this->create_publisher<situational_graphs_msgs::msg::CompleteRoomsData>(
        "complete_room_segmentation/complete_room_data", 1);
    room_data_pub = this->create_publisher<situational_graphs_msgs::msg::RoomsData>(
        "complete_room_segmentation/room_data", 1);
    room_centers_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "complete_room_segmentation/room_centers", 1);

    room_detection_timer = create_wall_timer(
        std::chrono::milliseconds(500), 
        std::bind(&TravelRoomSegmentationNode::room_detection_callback, this));
  }

  template <typename T>
  bool contains(std::vector<T> vec, const T& elem) {
    bool result = false;
    if (find(vec.begin(), vec.end(), elem) != vec.end()) {
      result = true;
    }
    return result;
  }

  // void map_planes_callback(
  //     const situational_graphs_msgs::msg::PlanesData::SharedPtr map_planes_msg) {
  //   std::lock_guard<std::mutex> lock(map_plane_mutex);
  //   if (!map_planes_msg->x_planes.empty()) x_vert_plane_queue.push_back(map_planes_msg->x_planes);
  //   if (!map_planes_msg->y_planes.empty()) y_vert_plane_queue.push_back(map_planes_msg->y_planes);
  // }

  void rooms_data_callback (
      const situational_graphs_msgs::msg::RoomsData::SharedPtr rooms_msg) {
    std::lock_guard<std::mutex> lock(room_mutex);
    for (auto &room : rooms_msg->rooms){
      if (!room.x_planes.empty()) x_vert_plane_queue.push_back(room.x_planes);
      if (!room.y_planes.empty()) y_vert_plane_queue.push_back(room.y_planes);
      subroom_vec.push_back(room);
    }
  }

  void flush_map_planes(
      std::vector<situational_graphs_msgs::msg::PlaneData>& current_x_vert_planes,
      std::vector<situational_graphs_msgs::msg::PlaneData>& current_y_vert_planes) {
    if (!x_vert_plane_queue.empty()){
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
        x_vert_plane_queue.pop_front();
      }
    }
    if (!y_vert_plane_queue.empty()){
      for (const auto& y_map_planes_msg : y_vert_plane_queue) {
        for (const auto& y_map_plane : y_map_planes_msg) {
          if (!contains(current_y_vert_planes, y_map_plane) ||
              current_y_vert_planes.empty()) {
            current_y_vert_planes.push_back(y_map_plane);
          } else {
            continue;
          }
        }
        y_vert_plane_queue.pop_front();
      }
    }
  }


  void corrected_odom_callback(
      const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg){
    // m_lock_odom.lock();
    odom_buf.push_back(*pose_msg);
  }

  void room_detection_callback() {
    auto t1 = this->now();
    extract_rooms();
    auto t2 = this->now();
    // std::cout << "duration to extract clusters: " << boost::format("%.3f") % (t2 -
    // t1).seconds() << std::endl;
    if (save_timings) {
      time_recorder.open("/tmp/room_seg_computation_time.txt",
                         std::ofstream::out | std::ofstream::app);
      time_recorder << std::to_string((t2 - t1).seconds()) + " \n";
      time_recorder.close();
    }
  }

  void local_traversable_edges_callback(
      const visualization_msgs::msg::MarkerArray::SharedPtr local_traversable_edges_msg) {
        local_traversable_edges = local_traversable_edges_msg;
        edges_cnt = 0;
        for (auto& marker: local_traversable_edges->markers)
          edges_cnt += marker.points.size();
  }

  /**
   * @brief extract clusters with its centers from the skeletal cloud
   *
   */
  void extract_rooms() {
    // flush_map_planes(current_x_vert_planes, current_y_vert_planes);
    std::vector<situational_graphs_msgs::msg::PlaneData> current_x_vert_planes,
        current_y_vert_planes;
    std::vector<situational_graphs_msgs::msg::CompleteRoomData> room_candidates_vec;    

    if (odom_buf.empty() || !local_traversable_edges) return;
    //set cloud hull square from last step
    visualization_msgs::msg::MarkerArray convex_hull_markers;
    if (square_cloud_hull && !square_cloud_hull->points.empty()){

      visualization_msgs::msg::Marker cloud_hull_marker;
      cloud_hull_marker.ns = "convex_hull";
      cloud_hull_marker.id = 0;
      cloud_hull_marker.header.stamp = this->now();
      cloud_hull_marker.header.frame_id = map_frame_id;
      cloud_hull_marker.action = visualization_msgs::msg::Marker::ADD;
      cloud_hull_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      cloud_hull_marker.scale.x = 0.3; cloud_hull_marker.scale.y = 0.3; cloud_hull_marker.scale.z = 0.3;
      cloud_hull_marker.color.a = 1.0;
      cloud_hull_marker.color.r = 1.0;
      cloud_hull_marker.color.g = 0.0;
      cloud_hull_marker.color.b = 0.0;
      cloud_hull_marker.pose.orientation.w = 1.0;
      for (int i = 0; i < (int) square_cloud_hull->points.size(); i++){
        geometry_msgs::msg::Point point;
        point.x = square_cloud_hull->points[i].x;
        point.y = square_cloud_hull->points[i].y;
        point.z = square_cloud_hull->points[i].z;
        cloud_hull_marker.points.push_back(point);
      }
      convex_hull_markers.markers.push_back(cloud_hull_marker);
      convex_hull_marker_pub->publish(convex_hull_markers);  
      
      room_analyzer->set_previous_global_cloud_hull(square_cloud_hull);
    }

    room_analyzer->set_odom_buf(odom_buf);
    room_analyzer->analyze_skeleton_graph(local_traversable_edges);      //////

    //current cloud in global frame (for subroom) 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr curr_cloud_cluster = room_analyzer->extract_cloud_cluster();  //current cluster inside room in global frame    
    if (curr_cloud_cluster && !curr_cloud_cluster->points.empty()) {
      //visualization current cluster in global frame
      sensor_msgs::msg::PointCloud2 current_cloud_cluster_msg;
      pcl::toROSMsg(*curr_cloud_cluster, current_cloud_cluster_msg);
      current_cloud_cluster_msg.header.stamp = this->now();
      current_cloud_cluster_msg.header.frame_id = map_frame_id;
      current_cluster_cloud_pub->publish(current_cloud_cluster_msg);

      for (auto &pt: curr_cloud_cluster->points){
        cloud_cluster->points.push_back(pt);         //global stacked clouds
      }

      // //visualization stacked clouds  
      // sensor_msgs::msg::PointCloud2 cloud_cluster_msg;
      // pcl::toROSMsg(*cloud_cluster, cloud_cluster_msg);
      // cloud_cluster_msg.header.stamp = this->now();
      // cloud_cluster_msg.header.frame_id = map_frame_id;
      // cluster_cloud_pub->publish(cloud_cluster_msg);

      //currnet cloud vis in global frame (for subroom)    
      visualization_msgs::msg::MarkerArray current_cloud_marker_array;// = room_analyzer->extract_marker_array_clusters(); //local??
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = map_frame_id;  // adjust to your frame
      marker.header.stamp = this->now();
      marker.ns = "points";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::POINTS;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.02;  // Point size in meters
      marker.scale.y = 0.02;

      for (const auto& point : curr_cloud_cluster->points) {
        geometry_msgs::msg::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        marker.points.push_back(p);
        std_msgs::msg::ColorRGBA color;
        color.r = 1;
        color.g = 1;
        color.b = 1;
        color.a = 1.0;  // Fully opaque
        marker.colors.push_back(color);
      }
      current_cloud_marker_array.markers.push_back(marker);
      current_cloud_marker_pub->publish(current_cloud_marker_array);
    }
    else if (!odom_buf.empty()) {
      pcl::PointXYZRGB pt;
      pt.x = odom_buf.back().pose.position.x;
      pt.y = odom_buf.back().pose.position.y;
      pt.z = odom_buf.back().pose.position.z;
      // cout<<"curr odom "<<pt.x<<" "<<pt.y<<endl;
      cloud_cluster->points.push_back(pt); 
    }
    odom_buf.clear();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZRGB>);
    float hull_area;
    if (edges_cnt<100) return;
    room_analyzer->extract_convex_hull(cloud_cluster, cloud_hull, hull_area);
    if (hull_area < 1.5) return;

    // visualize current pose
    visualization_msgs::msg::Marker curr_pose_marker = room_analyzer->get_cur_pose();
    curr_pose_marker.header.stamp = this->now();
    curr_pose_marker.header.frame_id = map_frame_id;
    curr_pose_pub->publish(curr_pose_marker);   

    bool is_room_completed = false; //room_analyzer->check_completion(); //after analyze_skeleton_graph
    std_msgs::msg::Bool is_room_completed_msg;
    is_room_completed_msg.data = is_room_completed;
    is_room_completed_pub->publish(is_room_completed_msg);

    if (is_room_completed) { // perform room segmentation
      // square_cloud_hulls.push_back(*square_cloud_hull);
      // std::cout<<"square_cloud_hulls.push_back"<<std::endl;

      situational_graphs_msgs::msg::CompleteRoomsData room_candidates_msg;
      // bool found_room = room_analyzer->perform_room_segmentation(
      //     cloud_cluster, subroom_vec, current_x_vert_planes, current_y_vert_planes, room_candidates_vec);
      // clear
      subroom_vec.clear();
      cloud_cluster->points.clear();
      square_cloud_hull->points.clear();
      // room visualization
      // room_candidates_msg.header.stamp = this->now();
      // room_candidates_msg.rooms = room_candidates_vec;
      // room_candidates_msg.convex_hull = convex_hull_markers;
      // complete_room_data_pub->publish(room_candidates_msg);
      // viz_room_centers(room_candidates_msg);
    }
    else{
      float min_x, min_y, max_x, max_y;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_square_cloud_hull(new pcl::PointCloud<pcl::PointXYZRGB>);
      room_analyzer->findBoundingBox(cloud_hull, min_x, min_y, max_x, max_y);
      room_analyzer->computeSmallestEnclosingSquare(tmp_square_cloud_hull, min_x, min_y, max_x, max_y);
      // std::cout<<"square hull "<<min_x<<" "<<max_x<<" "<<min_y<<" "<<max_y<<endl;
      // if (!square_cloud_hulls.empty()){
      //   pcl::PointCloud<pcl::PointXYZRGB>::Ptr overlap_convex_hull_pts(new pcl::PointCloud<pcl::PointXYZRGB>());
      //   overlap_convex_hull_pts->points.push_back(tmp_square_cloud_hull->points[0]);
      //   overlap_convex_hull_pts->points.push_back(tmp_square_cloud_hull->points[1]);
      //   overlap_convex_hull_pts->points.push_back(tmp_square_cloud_hull->points[2]);
      //   overlap_convex_hull_pts->points.push_back(tmp_square_cloud_hull->points[3]);
      //   for (auto &prev_hull: square_cloud_hulls){
      //     for (auto& pt: tmp_square_cloud_hull->points){
      //       if (pcl::isPointIn2DPolygon(pt, prev_hull)){
      //         std::cout<<"isPointIn2DPolygon"<<std::endl;
      //         overlap_convex_hull_pts->points.push_back(prev_hull.points[0]);
      //         overlap_convex_hull_pts->points.push_back(prev_hull.points[1]);
      //         overlap_convex_hull_pts->points.push_back(prev_hull.points[2]);
      //         overlap_convex_hull_pts->points.push_back(prev_hull.points[3]);
      //       }
      //     }
      //   }
      //   if (overlap_convex_hull_pts->points.size()>4) { 
      //     room_analyzer->findBoundingBox(overlap_convex_hull_pts, min_x, min_y, max_x, max_y);
      //     room_analyzer->computeSmallestEnclosingSquare(tmp_square_cloud_hull, min_x, min_y, max_x, max_y);
      //   }
      // }

      square_cloud_hull = tmp_square_cloud_hull;
      cloud_cluster->points.clear();
      for (auto &pt: tmp_square_cloud_hull->points){
        cloud_cluster->points.push_back(pt);         //global stacked clouds
      }
    }
   
  }

  void viz_room_centers(situational_graphs_msgs::msg::CompleteRoomsData room_vec) {
    visualization_msgs::msg::Marker room_marker;
    room_marker.pose.orientation.w = 1.0;
    room_marker.scale.x = 0.5;
    room_marker.scale.y = 0.5;
    room_marker.scale.z = 0.5;
    // plane_marker.points.resize(vert_planes.size());
    room_marker.header.frame_id = map_frame_id;
    room_marker.header.stamp = this->now();
    room_marker.ns = "complete_rooms";
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
      skeleton_graph_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      corrected_odom_sub;
  rclcpp::Subscription<situational_graphs_msgs::msg::PlanesData>::SharedPtr
      map_planes_sub;
  rclcpp::Subscription<situational_graphs_msgs::msg::RoomsData>::SharedPtr
      rooms_data_sub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr curr_pose_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr convex_hull_marker_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_room_completed_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr current_cluster_cloud_pub; //cluster_cloud_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr current_cloud_marker_pub;
  rclcpp::Publisher<situational_graphs_msgs::msg::CompleteRoomsData>::SharedPtr complete_room_data_pub;
  rclcpp::Publisher<situational_graphs_msgs::msg::RoomsData>::SharedPtr room_data_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr room_centers_pub;

  /* private variables */
 private:
  rclcpp::TimerBase::SharedPtr room_detection_timer;
  std::deque<geometry_msgs::msg::PoseStamped> odom_buf;
  int edges_cnt;

  std::mutex map_plane_mutex;
  std::mutex room_mutex;
  std::vector<situational_graphs_msgs::msg::PlaneData> x_vert_plane_vec,
      y_vert_plane_vec;
  std::deque<std::vector<situational_graphs_msgs::msg::PlaneData>> x_vert_plane_queue,
      y_vert_plane_queue;

  std::vector<situational_graphs_msgs::msg::RoomData> subroom_vec;

  std::unique_ptr<TravelRoomAnalyzer> room_analyzer;
  std::string map_frame_id = "map";
  bool save_timings;
  std::ofstream time_recorder;
 
 private:
  visualization_msgs::msg::MarkerArray::Ptr local_traversable_edges;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster; //global stacked cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr square_cloud_hull;
  // vector<pcl::PointCloud<pcl::PointXYZRGB>> square_cloud_hulls;
};

}  // namespace s_graphs

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<s_graphs::TravelRoomSegmentationNode>());
  rclcpp::shutdown();
  return 0;
}
