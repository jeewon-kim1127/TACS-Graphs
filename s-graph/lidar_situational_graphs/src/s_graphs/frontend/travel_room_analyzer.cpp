#include <s_graphs/frontend/travel_room_analyzer.hpp>

namespace s_graphs {

TravelRoomAnalyzer::TravelRoomAnalyzer(room_analyzer_params params, rclcpp::Clock::SharedPtr clock) {
  local_cloud_clusters.clear();
  // local_subgraph_centers.clear();
  // cloud_cluster = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
  
  odom_buf = std::deque<geometry_msgs::msg::PoseStamped>();
  vertex_neigh_thres = params.vertex_neigh_thres;
  
  tf_buffer = std::make_unique<tf2_ros::Buffer>(clock);
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  
  is_completed = false;
  complete_room_id = 0;
  cloud_hull_center.x = 0; cloud_hull_center.y = 0; cloud_hull_center.z = 0;
  disconnection = 0;
}

TravelRoomAnalyzer::~TravelRoomAnalyzer() {
  // cloud_cluster->points.clear();
  // local_cloud_cluster->points.clear();
  local_cloud_clusters.clear();
  // local_clusters_marker_array.markers.clear();
  // local_subgraph_centers.clear();
  odom_buf.clear();
}

void TravelRoomAnalyzer::analyze_skeleton_graph(  //local
    const visualization_msgs::msg::MarkerArray::SharedPtr& skeleton_graph_msg) {
  is_completed = false;
  local_cloud_clusters.clear();
  
  int subgraph_id = 0;
  
  visualization_msgs::msg::MarkerArray curr_connected_clusters;  
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> valid_cloud_clusters;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> curr_cloud_clusters;
  
  if (skeleton_graph_msg->markers.empty()) return;
  // visualization_msgs::msg::MarkerArray local_curr_connected_clusters;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_subgraph_centers(new pcl::PointCloud<pcl::PointXYZRGB>());
  for (const auto& single_graph : skeleton_graph_msg->markers) { //vertices, connected_vertices_, connected_edges_ .. several ns
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointXYZRGB center_pt; // center of single subgraph
    std::string vertex_string = "connected_edges_";
    size_t found = single_graph.ns.find(vertex_string);
    if (found != std::string::npos) {
      float r = rand() % 256;
      float g = rand() % 256;
      float b = rand() % 256;
      for (size_t i = 0; i < single_graph.points.size(); ++i) {
        pcl::PointXYZRGB pcl_point;
        pcl_point.x = single_graph.points[i].x;
        pcl_point.y = single_graph.points[i].y;
        pcl_point.z = 0.0;
        pcl_point.r = r;
        pcl_point.g = g;
        pcl_point.b = b;
        local_cloud_cluster->points.push_back(pcl_point);

        center_pt.x += pcl_point.x;
        center_pt.y += pcl_point.y;
        center_pt.z += pcl_point.z;
      }
      downsample_cloud_data(local_cloud_cluster);
      // insert subgraph id in the seq
      local_cloud_cluster->header.seq = subgraph_id;
      curr_cloud_clusters.push_back(local_cloud_cluster);
      curr_connected_clusters.markers.push_back(single_graph);
      subgraph_id++;
      // continue;
    }
    center_pt.x /= single_graph.points.size();
    center_pt.y /= single_graph.points.size();
    center_pt.z /= single_graph.points.size();
    local_subgraph_centers->points.push_back(center_pt);
    continue;
  }
  num_cloud_clusters = curr_cloud_clusters.size();
  //local
  // cout<<"curr_cloud_clusters size "<<curr_cloud_clusters.size()<<endl; 
  if (num_cloud_clusters==0) return ;

  local_cloud_clusters = curr_cloud_clusters; //only vertex
  local_clusters_marker_array = curr_connected_clusters; //connected edge&vertices

  try {
    tf_local2global = tf_buffer->lookupTransform(       //tf: local(skeleton) -> global(odom)
              odom_buf.front().header.frame_id, skeleton_graph_msg->markers[0].header.frame_id,  tf2::TimePointZero); 
    }catch (const tf2::TransformException& ex) {
    std::cout<< "Could not transform "<< skeleton_graph_msg->markers[0].header.frame_id<< " to "
                    <<odom_buf.front().header.frame_id<<std::endl;
    return;
  }
  Eigen::Affine3f transform = transformToEigen(tf_local2global);

  //check cluster enclosing current position

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr curr_cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>());
  if (num_cloud_clusters>=2 && previous_global_cloud_hull && !previous_global_cloud_hull->points.empty()){
    std::vector<int> outside_vector;
    for (int i=0; i<curr_cloud_clusters.size(); i++){
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_global_cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::transformPointCloud(*curr_cloud_clusters[i], *tmp_global_cloud_cluster, transform);
      
      int overlap_cnt = 0;
      for (auto &pt: tmp_global_cloud_cluster->points){
        if (pcl::isPointIn2DPolygon(pt, *previous_global_cloud_hull)) {
          overlap_cnt++;
          if (overlap_cnt>10) continue;
        }
      }
      // cout<<"overlap_cnt "<<i<<": "<<overlap_cnt<<endl;
      if (overlap_cnt<10) {    
        outside_vector.push_back(i); //index with less overlapping with previous convex hull
      }
      else{
        valid_cloud_clusters.push_back(curr_cloud_clusters[i]);
      }
    }

    if (!outside_vector.empty() && !odom_buf.empty()){
      current_odom = odom_buf.back();
      pcl::PointXYZRGB curr_position ;
      curr_position.x = current_odom.pose.position.x;
      curr_position.y = current_odom.pose.position.y;
      curr_position.z = current_odom.pose.position.z;
      // cur position visualization
      curr_position_marker.ns = "curr_position";
      curr_position_marker.id = 0;
      curr_position_marker.action = visualization_msgs::msg::Marker::ADD;
      curr_position_marker.type = visualization_msgs::msg::Marker::SPHERE;
      curr_position_marker.scale.x = 0.3; curr_position_marker.scale.y = 0.3; curr_position_marker.scale.z = 0.3;
      curr_position_marker.color.a = 1.0;
      curr_position_marker.color.r = 1.0;
      curr_position_marker.color.g = 1.0;
      curr_position_marker.color.b = 1.0;
      curr_position_marker.pose.orientation.w = 1.0;
      curr_position_marker.pose.position = current_odom.pose.position ;

      bool is_inside = pcl::isPointIn2DPolygon(curr_position, *previous_global_cloud_hull);
      if (!is_inside ){
        is_completed = true;
        complete_room_id ++;
        // std::cout<<"========is_completed====== room id "<<complete_room_id<<std::endl;
        previous_global_cloud_hull->points.clear();
        return;
      }
      if (!valid_cloud_clusters.empty()){
        for (int i=0; i<valid_cloud_clusters.size(); i++){
          if (std::find(outside_vector.begin(), outside_vector.end(),i)!=outside_vector.end()) continue;
          for (auto &pt: valid_cloud_clusters[i]->points)
            curr_cloud_cluster->points.push_back(pt);
        }
      }
    }
  }
  else {  // num_cloud_clusters == 1
    if (!previous_global_cloud_hull || previous_global_cloud_hull->points.empty()){
      for (auto &pt: curr_cloud_clusters[0]->points)
            curr_cloud_cluster->points.push_back(pt);
    }
    else {
      int overlap_cnt = 0;
      for (auto &pt: curr_cloud_clusters[0]->points){
        if (pcl::isPointIn2DPolygon(pt, *previous_global_cloud_hull)) {
          overlap_cnt++;
          if (overlap_cnt>10) break;;
        }
      }
      if (overlap_cnt>10) {
        for (auto &pt: curr_cloud_clusters[0]->points)
            curr_cloud_cluster->points.push_back(pt);
      }
    }
  }

  if (!curr_cloud_cluster->points.empty()){  
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*curr_cloud_cluster, *global_cloud_cluster, transform);
    cloud_cluster = global_cloud_cluster;
  }
  else{ //empty curr_cloud_cluster
    cloud_cluster = curr_cloud_cluster; 
  }
  return;
}
//////////////////////////////////////////
//subroom
// Function to convert geometry_msgs::msg::Vector3[] to pcl::PointCloud<pcl::PointXYZRGB>::Ptr
// pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertVector3ArrayToPointCloud(const std::vector<geometry_msgs::msg::Vector3>& vector_array)
// {
//     auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
//     cloud->width = vector_array.size();
//     cloud->height = 1;
//     cloud->is_dense = false;
//     cloud->points.resize(vector_array.size());

//     for (size_t i = 0; i < vector_array.size(); ++i)
//     {
//         pcl::PointXYZRGB point;
//         point.x = vector_array[i].x;
//         point.y = vector_array[i].y;
//         point.z = vector_array[i].z;
//         // Set a default color (e.g., white)
//         point.r = 255;
//         point.g = 255;
//         point.b = 255;
//         cloud->points[i] = point;
//     }
//     return cloud;
// }
geometry_msgs::msg::Point TravelRoomAnalyzer::extract_room_length(const pcl::PointXY& p1,
                                                            const pcl::PointXY& p2) {
  geometry_msgs::msg::Point length;
  if (fabs(p1.x) > fabs(p2.x)) {
    length.x = fabs(p1.x - p2.x);
  } else {
    length.x = fabs(p2.x - p1.x);
  }

  if (fabs(p1.y) > fabs(p2.y)) {
    length.y = fabs(p1.y - p2.y);
  } else {
    length.y = fabs(p2.y - p1.y);
  }
  return length;
}

void TravelRoomAnalyzer::extract_cluster_endpoints(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& skeleton_cloud,
    pcl::PointXY& p1,
    pcl::PointXY& p2) {
  pcl::PointXYZRGB min, max;
  pcl::getMinMax3D(*skeleton_cloud, min, max);
  p1.x = min.x;
  p1.y = min.y;
  p2.x = max.x;
  p2.y = max.y;
}

bool TravelRoomAnalyzer::extract_centroid_location(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& skeleton_cloud,
    const pcl::PointXY& p1,
    const pcl::PointXY& p2) {
  pcl::PointXYZRGB min, max;
  min.x = p1.x;
  min.y = p1.y;
  max.x = p2.x;
  max.y = p2.y;

  pcl::PointXYZRGB centroid = compute_centroid(min, max);

  // get the dist of centroid wrt to all the points in the cluster
  // centroids lying outside will have higher distance to the points in the cluster
  float min_dist = 100;
  for (size_t i = 0; i < skeleton_cloud->points.size(); ++i) {
    float dist = sqrt(pow(centroid.x - skeleton_cloud->points[i].x, 2) +
                      pow(centroid.y - skeleton_cloud->points[i].y, 2));

    if (dist < min_dist) {
      min_dist = dist;
    }
  }

  if (min_dist > 0.5) {
    // std::cout << "centroid outside the cluster! Do something " << std::endl;
    return false;
  }

  return true;
}

bool TravelRoomAnalyzer::extract_centroid_location(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& skeleton_cloud,
    const geometry_msgs::msg::Point& room_center) {
  // get the dist of centroid wrt to all the points in the cluster
  // centroids lying outside will have higher distance to the points in the cluster
  float min_dist = 100;
  for (size_t i = 0; i < skeleton_cloud->points.size(); ++i) {
    float dist = sqrt(pow(room_center.x - skeleton_cloud->points[i].x, 2) +
                      pow(room_center.y - skeleton_cloud->points[i].y, 2));

    if (dist < min_dist) {
      min_dist = dist;
    }
  }

  if (min_dist > 0.5) {
    // std::cout << "centroid outside the cluster! Discarding the found room " <<
    // std::endl;
    return false;
  }

  return true;
}

pcl::PointXYZRGB TravelRoomAnalyzer::compute_centroid(const pcl::PointXYZRGB& p1,
                                                const pcl::PointXYZRGB& p2) {
  pcl::PointXYZRGB center;
  if (fabs(p1.x) > fabs(p2.x)) {
    float size = p1.x - p2.x;
    center.x = (size / 2) + p2.x;
  } else {
    float size = p2.x - p1.x;
    center.x = (size / 2) + p1.x;
  }

  if (fabs(p1.y) > fabs(p2.y)) {
    float size = p1.y - p2.y;
    center.y = (size / 2) + p2.y;
  } else {
    float size = p2.y - p1.y;
    center.y = (size / 2) + p1.y;
  }

  center.z = p1.z;
  return center;
}

double dist(pcl::PointXYZRGB start, pcl::PointXYZRGB end){
  return sqrt(pow(start.x-end.x,2)+pow(start.y-end.y,2)+pow(start.z-end.z,2));
}

void TravelRoomAnalyzer::extract_convex_hull(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr skeleton_cloud,  //global
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_hull,
    float& area) {
  if (skeleton_cloud->points.empty()) return;

  pcl::ConvexHull<pcl::PointXYZRGB> convex_hull;
  std::vector<pcl::Vertices> polygons;
  pcl::PointXYZRGB additional_pt;
  
  additional_pt.x = skeleton_cloud->points[0].x;additional_pt.y = skeleton_cloud->points[0].y; additional_pt.z = skeleton_cloud->points[0].z+1.0;
  skeleton_cloud->points.push_back(additional_pt);
  convex_hull.setInputCloud(skeleton_cloud);
  convex_hull.setDimension(2);
  convex_hull.setComputeAreaVolume(true);
  convex_hull.reconstruct(*cloud_hull, polygons);
  // std::cout << "polygons before: " << polygons[0].vertices.size() << std::endl;
  area = convex_hull.getTotalArea();

  return;
}
bool TravelRoomAnalyzer::perform_room_segmentation(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster,
    std::vector<situational_graphs_msgs::msg::RoomData>& subrooms_vec,
    const std::vector<situational_graphs_msgs::msg::PlaneData>& current_x_vert_planes,
    const std::vector<situational_graphs_msgs::msg::PlaneData>& current_y_vert_planes,
    std::vector<situational_graphs_msgs::msg::CompleteRoomData>& room_candidate_vec) {
  
  pcl::PointXY p1;
  pcl::PointXY p2;
  extract_cluster_endpoints(cloud_cluster, p1, p2);
  float room_width_threshold = 1.0;
  geometry_msgs::msg::Point room_length = extract_room_length(p1, p2);
  if (room_length.x < 0.5 || room_length.y < 0.5) 
    return false;

  // analyze the largest x plane pair
  situational_graphs_msgs::msg::PlaneData room_x_plane1, room_x_plane2;
  situational_graphs_msgs::msg::PlaneData room_y_plane1, room_y_plane2;
  room_x_plane1.nx = -1;
  room_x_plane2.nx = -1;
  room_y_plane1.nx = -1;
  room_y_plane2.nx = -1;

  float max_xplane_width = 0;
  for (auto x_plane1 : current_x_vert_planes) {
    if (x_plane1.nx < 0) {
      continue;
    }
    PlaneUtils::correct_plane_direction(PlaneUtils::plane_class::X_VERT_PLANE,
                                        x_plane1);

    for (auto x_plane2 : current_x_vert_planes) {
      if (x_plane2.nx > 0) {
        continue;
      }
      PlaneUtils::correct_plane_direction(PlaneUtils::plane_class::X_VERT_PLANE,
                                          x_plane2);

      float x_plane_width = PlaneUtils::width_between_planes(x_plane1, x_plane2);
      bool planes_placed_correctly = false;
      if (!x_plane1.plane_points.empty() && !x_plane2.plane_points.empty()) {
        planes_placed_correctly = PlaneUtils::compute_point_difference(
            x_plane1.plane_points.back().x, x_plane2.plane_points.back().x);
      } else
        continue;

      float dot_product = PlaneUtils::plane_dot_product(x_plane1, x_plane2);
      if (abs(dot_product) < 0.9) continue;

      if (x_plane_width > max_xplane_width && planes_placed_correctly) {
        max_xplane_width = x_plane_width;
        room_x_plane1 = x_plane1;
        room_x_plane2 = x_plane2;
      }
    }
  }
  float max_yplane_width = 0;
  for (auto y_plane1 : current_y_vert_planes) {
    if (y_plane1.ny < 0) {
      continue;
    }
    PlaneUtils::correct_plane_direction(PlaneUtils::plane_class::Y_VERT_PLANE,
                                        y_plane1);

    for (auto y_plane2 : current_y_vert_planes) {
      if (y_plane2.ny > 0) {
        continue;
      }
      PlaneUtils::correct_plane_direction(PlaneUtils::plane_class::Y_VERT_PLANE,
                                          y_plane2);

      float y_plane_width = PlaneUtils::width_between_planes(y_plane1, y_plane2);
      bool planes_placed_correctly = false;
      if (!y_plane1.plane_points.empty() && !y_plane2.plane_points.empty()) {
        planes_placed_correctly = PlaneUtils::compute_point_difference(
            y_plane1.plane_points.back().y, y_plane2.plane_points.back().y);
      } else
        continue;

      float dot_product = PlaneUtils::plane_dot_product(y_plane1, y_plane2);
      if (abs(dot_product) < 0.9) continue;

      if (y_plane_width > max_yplane_width && planes_placed_correctly) {
        max_yplane_width = y_plane_width;
        room_y_plane1 = y_plane1;
        room_y_plane2 = y_plane2;
      }
    }
  }
  if (room_x_plane1.nx == -1 || room_x_plane2.nx != -1|| room_y_plane1.nx != -1 || room_y_plane2.nx != -1)
    return false;

  if (!is_x1_plane_aligned_w_y(room_x_plane1.plane_points, room_y_plane1.plane_points) ||
      !is_x1_plane_aligned_w_y(room_x_plane1.plane_points, room_y_plane2.plane_points)) {
    // std::cout << "returning as not a valid room configuration" << std::endl;
    return false;
  }
  if (!is_x2_plane_aligned_w_y(room_x_plane2.plane_points, room_y_plane1.plane_points) ||
      !is_x2_plane_aligned_w_y(room_x_plane2.plane_points, room_y_plane2.plane_points)) {
    // std::cout << "returning as not a valid room configuration" << std::endl;
    return false;
  }
  if (!is_y2_plane_aligned_w_x(room_y_plane2.plane_points, room_x_plane1.plane_points) ||
      !is_y2_plane_aligned_w_x(room_y_plane2.plane_points, room_x_plane2.plane_points)) {
    // std::cout << "returning as not a valid room configuration" << std::endl;
    return false;
  }
  geometry_msgs::msg::Pose room_center =
      PlaneUtils::room_center(room_x_plane1, room_x_plane2, room_y_plane1, room_y_plane2);
  bool centroid_inside =
      extract_centroid_location(cloud_cluster, room_center.position);
  if (!centroid_inside) {
    // std::cout << "returning as the room center is outside the cluster" << std::endl;
    return false;
  }

  situational_graphs_msgs::msg::CompleteRoomData room_candidate;
  // room_candidate.header = cloud_cluster->header;
  room_candidate.id = cloud_cluster->header.seq;
  room_candidate.room_length = room_length;
  room_candidate.room_center = room_center;
  room_candidate.x_planes.push_back(room_x_plane1);
  room_candidate.x_planes.push_back(room_x_plane2);
  room_candidate.y_planes.push_back(room_y_plane1);
  room_candidate.y_planes.push_back(room_y_plane2);
  room_candidate.subrooms = subrooms_vec;
  // room_candidate.convex_hull = cloud_cluster;

  room_candidate_vec.push_back(room_candidate);
  return true;
}
/*
bool TravelRoomAnalyzer::perform_room_segmentation(
    RoomInfo& room_info,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_in,
    std::vector<situational_graphs_msgs::msg::RoomData>& room_candidates_vec,
    const visualization_msgs::msg::MarkerArray& cloud_marker_array) {
  pcl::PointXY p1;
  pcl::PointXY p2;
  extract_cluster_endpoints(cloud_cluster_in, p1, p2);
  float room_width_threshold = 1.0;
  geometry_msgs::msg::Point room_length = extract_room_length(p1, p2);

  if (room_length.x < 0.5 || room_length.y < 0.5) {
    return false;
  } else {
    // check how many planes are extracted
    // if four planes are found its a bounded room
    // if 2 parallel planes are found it an ifnite corridor

    situational_graphs_msgs::msg::PlaneData x_plane1;
    x_plane1.nx = 0;
    x_plane1.ny = 0;
    x_plane1.nz = 0;
    situational_graphs_msgs::msg::PlaneData x_plane2;
    x_plane2.nx = 0;
    x_plane2.ny = 0;
    x_plane2.nz = 0;
    situational_graphs_msgs::msg::PlaneData y_plane1;
    y_plane1.nx = 0;
    y_plane1.ny = 0;
    y_plane1.nz = 0;
    situational_graphs_msgs::msg::PlaneData y_plane2;
    y_plane2.nx = 0;
    y_plane2.ny = 0;
    y_plane1.nz = 0;
    RoomPlanes room_planes = {
        x_plane1, x_plane2, y_plane1, y_plane2, false, false, false, false};
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sub_cloud_cluster =
        extract_room_planes(room_info, room_planes, p1, p2);

    // if found all four planes its a room
    if (room_planes.found_x1_plane && room_planes.found_x2_plane &&
        room_planes.found_y1_plane && room_planes.found_y2_plane) {
      PlaneUtils::correct_plane_direction(PlaneUtils::plane_class::X_VERT_PLANE,
                                          room_planes.x_plane1);
      PlaneUtils::correct_plane_direction(PlaneUtils::plane_class::X_VERT_PLANE,
                                          room_planes.x_plane2);
      PlaneUtils::correct_plane_direction(PlaneUtils::plane_class::Y_VERT_PLANE,
                                          room_planes.y_plane1);
      PlaneUtils::correct_plane_direction(PlaneUtils::plane_class::Y_VERT_PLANE,
                                          room_planes.y_plane2);

      // first check the width of the rooms
      float x_plane_width =
          PlaneUtils::width_between_planes(room_planes.x_plane1, room_planes.x_plane2);
      float y_plane_width =
          PlaneUtils::width_between_planes(room_planes.y_plane1, room_planes.y_plane2);

      if (x_plane_width < room_width_threshold ||
          y_plane_width < room_width_threshold) {
        // std::cout << "returning as the room is not sufficiently wide" << std::endl;
        return false;
      }

      if (!is_x1_plane_aligned_w_y(x_plane1.plane_points, y_plane1.plane_points) ||
          !is_x1_plane_aligned_w_y(x_plane1.plane_points, y_plane2.plane_points)) {
        // std::cout << "returning as not a valid room configuration" << std::endl;
        return false;
      }

      if (!is_x2_plane_aligned_w_y(x_plane2.plane_points, y_plane1.plane_points) ||
          !is_x2_plane_aligned_w_y(x_plane2.plane_points, y_plane2.plane_points)) {
        // std::cout << "returning as not a valid room configuration" << std::endl;
        return false;
      }

      if (!is_y2_plane_aligned_w_x(y_plane2.plane_points, x_plane1.plane_points) ||
          !is_y2_plane_aligned_w_x(y_plane2.plane_points, x_plane2.plane_points)) {
        // std::cout << "returning as not a valid room configuration" << std::endl;
        return false;
      }
      geometry_msgs::msg::Pose room_center =
          PlaneUtils::room_center(x_plane1, x_plane2, y_plane1, y_plane2);
      bool centroid_inside =
          extract_centroid_location(cloud_cluster_in, room_center.position);
      if (!centroid_inside) {
        // std::cout << "returning as the room center is outside the cluster" <<
        // std::endl;
        return false;
      }

      // clear plane points which are not required now
      room_planes.x_plane1.plane_points.clear();
      room_planes.x_plane2.plane_points.clear();
      room_planes.y_plane1.plane_points.clear();
      room_planes.y_plane2.plane_points.clear();

      situational_graphs_msgs::msg::RoomData room_candidate;
      room_candidate.id = cloud_cluster_in->header.seq;
      room_candidate.room_length = room_length;
      room_candidate.room_center = room_center;
      room_candidate.x_planes.push_back(room_planes.x_plane1);
      room_candidate.x_planes.push_back(room_planes.x_plane2);
      room_candidate.y_planes.push_back(room_planes.y_plane1);
      room_candidate.y_planes.push_back(room_planes.y_plane2);
      for (int i = 0; i < cloud_marker_array.markers.size(); ++i) {
        room_candidate.cluster_array.markers.push_back(cloud_marker_array.markers[i]);
      }
      room_candidates_vec.push_back(room_candidate);

      return true;
    }
    // if found only two x planes are found add x infinite_room
    else if (room_planes.found_x1_plane && room_planes.found_x2_plane &&
             (!room_planes.found_y1_plane || !room_planes.found_y2_plane)) {
      if (sub_cloud_cluster->points.size() > 0)
        extract_cluster_endpoints(sub_cloud_cluster, p1, p2);

      PlaneUtils::correct_plane_direction(PlaneUtils::plane_class::X_VERT_PLANE,
                                          room_planes.x_plane1);
      PlaneUtils::correct_plane_direction(PlaneUtils::plane_class::X_VERT_PLANE,
                                          room_planes.x_plane2);

      float x_plane_width =
          PlaneUtils::width_between_planes(room_planes.x_plane1, room_planes.x_plane2);
      if (x_plane_width < room_width_threshold) {
        // std::cout << "returning as the room is not sufficiently wide" << std::endl;
        return false;
      }

      Eigen::Vector2d cluster_center;
      geometry_msgs::msg::Pose room_center =
          PlaneUtils::extract_infite_room_center(PlaneUtils::plane_class::X_VERT_PLANE,
                                                 p1,
                                                 p2,
                                                 room_planes.x_plane1,
                                                 room_planes.x_plane2,
                                                 cluster_center);
      bool centroid_inside =
          extract_centroid_location(cloud_cluster_in, room_center.position);
      if (!centroid_inside) {
        // std::cout << "returning as the room center is outside the cluster" <<
        // std::endl;
        return false;
      }

      // clear plane points which are not required now
      room_planes.x_plane1.plane_points.clear();
      room_planes.x_plane2.plane_points.clear();
      room_planes.y_plane1.plane_points.clear();
      room_planes.y_plane2.plane_points.clear();

      situational_graphs_msgs::msg::RoomData room_candidate;
      room_candidate.id = cloud_cluster_in->header.seq;
      room_candidate.room_length = room_length;
      room_candidate.room_center = room_center;
      room_candidate.cluster_center.x = cluster_center(0);
      room_candidate.cluster_center.y = cluster_center(1);
      room_candidate.x_planes.push_back(room_planes.x_plane1);
      room_candidate.x_planes.push_back(room_planes.x_plane2);
      for (int i = 0; i < cloud_marker_array.markers.size(); ++i) {
        room_candidate.cluster_array.markers.push_back(cloud_marker_array.markers[i]);
      }
      room_candidates_vec.push_back(room_candidate);

      return true;
    }
    // if found only two y planes are found at y infinite_room
    else if (room_planes.found_y1_plane && room_planes.found_y2_plane &&
             (!room_planes.found_x1_plane || !room_planes.found_x2_plane)) {
      if (sub_cloud_cluster->points.size() > 0)
        extract_cluster_endpoints(sub_cloud_cluster, p1, p2);

      PlaneUtils::correct_plane_direction(PlaneUtils::plane_class::Y_VERT_PLANE,
                                          room_planes.y_plane1);
      PlaneUtils::correct_plane_direction(PlaneUtils::plane_class::Y_VERT_PLANE,
                                          room_planes.y_plane2);

      float y_plane_width =
          PlaneUtils::width_between_planes(room_planes.y_plane1, room_planes.y_plane2);
      if (y_plane_width < room_width_threshold) {
        // std::cout << "returning as the room is not sufficiently wide" << std::endl;
        return false;
      }

      Eigen::Vector2d cluster_center;
      geometry_msgs::msg::Pose room_center =
          PlaneUtils::extract_infite_room_center(PlaneUtils::plane_class::Y_VERT_PLANE,
                                                 p1,
                                                 p2,
                                                 room_planes.y_plane1,
                                                 room_planes.y_plane2,
                                                 cluster_center);
      bool centroid_inside =
          extract_centroid_location(cloud_cluster_in, room_center.position);
      if (!centroid_inside) {
        // std::cout << "returning as the room center is outside the cluster" <<
        // std::endl;
        return false;
      }

      // clear plane points which are not required now
      room_planes.x_plane1.plane_points.clear();
      room_planes.x_plane2.plane_points.clear();
      room_planes.y_plane1.plane_points.clear();
      room_planes.y_plane2.plane_points.clear();

      situational_graphs_msgs::msg::RoomData room_candidate;
      room_candidate.id = cloud_cluster_in->header.seq;
      room_candidate.room_length = room_length;
      room_candidate.room_center = room_center;
      room_candidate.cluster_center.x = cluster_center(0);
      room_candidate.cluster_center.y = cluster_center(1);
      room_candidate.y_planes.push_back(room_planes.y_plane1);
      room_candidate.y_planes.push_back(room_planes.y_plane2);
      for (int i = 0; i < cloud_marker_array.markers.size(); ++i) {
        room_candidate.cluster_array.markers.push_back(cloud_marker_array.markers[i]);
      }
      room_candidates_vec.push_back(room_candidate);
      return true;
    } else {
      return false;
    }
  }
  return false;
}
*/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr TravelRoomAnalyzer::extract_room_planes(
    RoomInfo& room_info,
    RoomPlanes& room_planes,
    pcl::PointXY p_min,
    pcl::PointXY p_max) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_x1(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_x2(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_y1(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_y2(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr sub_cloud_cluster(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  int max_x1_neighbours, max_x2_neighbours, max_y1_neighbours, max_y2_neighbours;
  max_x1_neighbours = max_x2_neighbours = max_y1_neighbours = max_y2_neighbours = 0;
  int min_neighbors_thres = vertex_neigh_thres;

  pcl::PointXY top_right, bottom_right, top_left, bottom_left;

  int point_cloud_size_thres = 500;
  if (room_info.cloud_cluster->points.size() > point_cloud_size_thres) {
    downsample_cloud_data(room_info.cloud_cluster);
  }

  for (const auto& x_plane : room_info.current_x_vert_planes) {
    if (x_plane.nx < 0) {
      continue;
    }

    int x1_neighbours =
        find_plane_points(room_info.cloud_cluster, x_plane, cloud_cluster_x1);

    if (x1_neighbours > max_x1_neighbours) {
      max_x1_neighbours = x1_neighbours;
      room_planes.x_plane1 = x_plane;
    }
  }

  for (const auto& x_plane : room_info.current_x_vert_planes) {
    if (x_plane.nx > 0) {
      continue;
    }

    int x2_neighbours =
        find_plane_points(room_info.cloud_cluster, x_plane, cloud_cluster_x2);

    if (x2_neighbours > max_x2_neighbours) {
      max_x2_neighbours = x2_neighbours;
      room_planes.x_plane2 = x_plane;
    }
  }

  if (room_planes.x_plane1.nx * room_planes.x_plane2.nx >= 0) {
    room_planes.found_x1_plane = false, room_planes.found_x2_plane = false;
  } else {
    bool planes_placed_correctly = false;
    if (!room_planes.x_plane1.plane_points.empty() &&
        !room_planes.x_plane2.plane_points.empty()) {
      planes_placed_correctly = PlaneUtils::compute_point_difference(
          room_planes.x_plane1.plane_points.back().x,
          room_planes.x_plane2.plane_points.back().x);
    }

    if (max_x1_neighbours >= min_neighbors_thres && planes_placed_correctly) {
      room_planes.found_x1_plane = true;
    } else {
      room_planes.found_x1_plane = false;
    }
    if (max_x2_neighbours >= min_neighbors_thres && planes_placed_correctly) {
      room_planes.found_x2_plane = true;
    } else {
      room_planes.found_x2_plane = false;
    }
  }

  for (const auto& y_plane : room_info.current_y_vert_planes) {
    if (y_plane.ny < 0) {
      continue;
    }
    float dist_y1 = -1 * (y_plane.nx * p_min.x + y_plane.ny * p_min.y);
    float diff_dist_y1 = 100;
    diff_dist_y1 = sqrt((dist_y1 - y_plane.d) * (dist_y1 - y_plane.d));

    int y1_neighbours =
        find_plane_points(room_info.cloud_cluster, y_plane, cloud_cluster_y1);

    if (y1_neighbours > max_y1_neighbours) {
      max_y1_neighbours = y1_neighbours;
      room_planes.y_plane1 = y_plane;
    }
  }

  for (const auto& y_plane : room_info.current_y_vert_planes) {
    if (y_plane.ny > 0) {
      continue;
    }
    float dist_y2 = -1 * (y_plane.nx * p_max.x + y_plane.ny * p_max.y);
    float diff_dist_y2 = 100;
    diff_dist_y2 = sqrt((dist_y2 - y_plane.d) * (dist_y2 - y_plane.d));

    int y2_neighbours =
        find_plane_points(room_info.cloud_cluster, y_plane, cloud_cluster_y2);

    if (y2_neighbours > max_y2_neighbours) {
      max_y2_neighbours = y2_neighbours;
      room_planes.y_plane2 = y_plane;
    }
  }

  if (room_planes.y_plane1.ny * room_planes.y_plane2.ny >= 0) {
    room_planes.found_y1_plane = false, room_planes.found_y2_plane = false;
  } else {
    bool planes_placed_correctly = false;
    if (!room_planes.y_plane1.plane_points.empty() &&
        !room_planes.y_plane2.plane_points.empty()) {
      planes_placed_correctly = PlaneUtils::compute_point_difference(
          room_planes.y_plane1.plane_points.back().y,
          room_planes.y_plane2.plane_points.back().y);
    }

    if (max_y1_neighbours >= min_neighbors_thres && planes_placed_correctly) {
      room_planes.found_y1_plane = true;
    } else {
      room_planes.found_y1_plane = false;
    }
    if (max_y2_neighbours >= min_neighbors_thres && planes_placed_correctly) {
      room_planes.found_y2_plane = true;
    } else {
      room_planes.found_y2_plane = false;
    }
  }

  if (room_planes.found_x1_plane && room_planes.found_x2_plane &&
      room_planes.found_y1_plane && room_planes.found_y2_plane) {
    for (int i = 0; i < cloud_cluster_x1->points.size(); ++i)
      sub_cloud_cluster->points.push_back(cloud_cluster_x1->points[i]);
    for (int i = 0; i < cloud_cluster_x2->points.size(); ++i)
      sub_cloud_cluster->points.push_back(cloud_cluster_x2->points[i]);
    for (int i = 0; i < cloud_cluster_y1->points.size(); ++i)
      sub_cloud_cluster->points.push_back(cloud_cluster_y1->points[i]);
    for (int i = 0; i < cloud_cluster_y2->points.size(); ++i)
      sub_cloud_cluster->points.push_back(cloud_cluster_y2->points[i]);
  } else if (room_planes.found_x1_plane && room_planes.found_x2_plane &&
             (!room_planes.found_y1_plane || !room_planes.found_y2_plane)) {
    for (int i = 0; i < cloud_cluster_x1->points.size(); ++i)
      sub_cloud_cluster->points.push_back(cloud_cluster_x1->points[i]);
    for (int i = 0; i < cloud_cluster_x2->points.size(); ++i)
      sub_cloud_cluster->points.push_back(cloud_cluster_x2->points[i]);
  } else if (room_planes.found_y1_plane && room_planes.found_y2_plane &&
             (!room_planes.found_x1_plane || !room_planes.found_x2_plane)) {
    for (int i = 0; i < cloud_cluster_y1->points.size(); ++i)
      sub_cloud_cluster->points.push_back(cloud_cluster_y1->points[i]);
    for (int i = 0; i < cloud_cluster_y2->points.size(); ++i)
      sub_cloud_cluster->points.push_back(cloud_cluster_y2->points[i]);
  }

  return sub_cloud_cluster;
}

bool TravelRoomAnalyzer::is_x1_plane_aligned_w_y(
    const std::vector<geometry_msgs::msg::Vector3> x_plane1_points,
    const std::vector<geometry_msgs::msg::Vector3> y_plane_points) {
  bool valid_room_config = false;
  int point_count = 0;
  for (int i = 0; i < y_plane_points.size(); ++i) {
    for (int j = 0; j < x_plane1_points.size(); ++j) {
      float dist = y_plane_points[i].x - x_plane1_points[j].x;
      if (dist > 0) {
        point_count++;
        break;
      }
    }
    if (point_count > 500) {
      valid_room_config = true;
      break;
    }
  }
  return valid_room_config;
}

bool TravelRoomAnalyzer::is_x2_plane_aligned_w_y(
    const std::vector<geometry_msgs::msg::Vector3> x_plane2_points,
    const std::vector<geometry_msgs::msg::Vector3> y_plane_points) {
  bool valid_room_config = false;
  int point_count = 0;
  for (int i = 0; i < y_plane_points.size(); ++i) {
    for (int j = 0; j < x_plane2_points.size(); ++j) {
      float dist = y_plane_points[i].x - x_plane2_points[j].x;
      if (dist < 0) {
        point_count++;
        break;
      }
    }
    if (point_count > 500) {
      valid_room_config = true;
      break;
    }
  }
  return valid_room_config;
}

bool TravelRoomAnalyzer::is_y1_plane_aligned_w_x(
    const std::vector<geometry_msgs::msg::Vector3> y_plane1_points,
    const std::vector<geometry_msgs::msg::Vector3> x_plane_points) {
  bool valid_room_config = false;
  int point_count = 0;
  for (int i = 0; i < x_plane_points.size(); ++i) {
    for (int j = 0; j < y_plane1_points.size(); ++j) {
      float dist = x_plane_points[i].y - y_plane1_points[j].y;
      if (dist > 0) {
        point_count++;
        break;
      }
    }
    if (point_count > 500) {
      valid_room_config = true;
      break;
    }
  }
  return valid_room_config;
}

bool TravelRoomAnalyzer::is_y2_plane_aligned_w_x(
    const std::vector<geometry_msgs::msg::Vector3> y_plane2_points,
    const std::vector<geometry_msgs::msg::Vector3> x_plane_points) {
  bool valid_room_config = false;
  int point_count = 0;
  for (int i = 0; i < x_plane_points.size(); ++i) {
    for (int j = 0; j < y_plane2_points.size(); ++j) {
      float dist = x_plane_points[i].y - y_plane2_points[j].y;
      if (dist < 0) {
        point_count++;
        break;
      }
    }
    if (point_count > 500) {
      valid_room_config = true;
      break;
    }
  }
  return valid_room_config;
}

std::vector<float> TravelRoomAnalyzer::find_plane_points(
    const pcl::PointXY& start_point,
    const pcl::PointXY& end_point,
    const situational_graphs_msgs::msg::PlaneData& plane) {
  float min_start_point_plane_dist = 100;
  float min_end_point_plane_dist = 100;
  std::vector<float> plane_point_distances;
  geometry_msgs::msg::Vector3 closest_start_plane_point, closest_end_plane_point;
  for (const auto& plane_point : plane.plane_points) {
    float start_plane_point_dist = sqrt(pow(start_point.x - plane_point.x, 2) +
                                        pow(start_point.y - plane_point.y, 2));

    float end_plane_point_dist =
        sqrt(pow(end_point.x - plane_point.x, 2) + pow(end_point.y - plane_point.y, 2));

    if (start_plane_point_dist < min_start_point_plane_dist) {
      min_start_point_plane_dist = start_plane_point_dist;
      closest_start_plane_point = plane_point;
    }

    if (end_plane_point_dist < min_end_point_plane_dist) {
      min_end_point_plane_dist = end_plane_point_dist;
      closest_end_plane_point = plane_point;
    }
  }

  plane_point_distances.push_back(min_start_point_plane_dist);
  plane_point_distances.push_back(min_end_point_plane_dist);

  return plane_point_distances;
}

void TravelRoomAnalyzer::downsample_cloud_data(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_hull) {
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud(cloud_hull);
  sor.setLeafSize(0.1f, 0.1f, 0.1f);
  sor.filter(*cloud_hull);
}

int TravelRoomAnalyzer::find_plane_points(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_hull,
    const situational_graphs_msgs::msg::PlaneData& plane,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& sub_cloud_cluster) {
  int num_neighbours = 0;
  double point_hull_dist_thres = 1.0;

  for (int i = 0; i < cloud_hull->points.size(); ++i) {
    double min_point_hull_dist = 1000;
    for (const auto& plane_point : plane.plane_points) {
      float point_hull_dist = sqrt(pow(plane_point.x - cloud_hull->points[i].x, 2) +
                                   pow(plane_point.y - cloud_hull->points[i].y, 2));
      if (point_hull_dist < min_point_hull_dist) {
        min_point_hull_dist = point_hull_dist;
      }
    }
    if (min_point_hull_dist < point_hull_dist_thres) {
      num_neighbours++;
      sub_cloud_cluster->points.push_back(cloud_hull->points[i]);
    }
  }
  // std::cout << "num nieghbours: " << num_neighbours << std::endl;
  return num_neighbours;
}

}  // namespace s_graphs
