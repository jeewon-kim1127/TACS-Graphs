#include "s_graphs/common/room_utils.hpp"

/**
 * Obtain the vertical planes that form the boundaries of a given room.
 * @param room The room whose planes are to be obtained.
 * @param x_vert_planes The list of vertical planes aligned with the x-axis.
 * @param y_vert_planes The list of vertical planes aligned with the y-axis.
 * @return A vector of pointers to the vertical planes that form the boundaries of the
 * room.
 */


bool  is_x1_plane_aligned_w_y(
    const pcl::PointCloud<PointNormal>::Ptr x_plane1_points,
    const pcl::PointCloud<PointNormal>::Ptr y_plane_points) {
  bool valid_room_config = false;
  int point_count = 0;
  int size = y_plane_points->points.size();
  for (int i = 0; i < y_plane_points->points.size(); ++i) {
    for (int j = 0; j < x_plane1_points->points.size(); ++j) {
      float dist = y_plane_points->points[i].x - x_plane1_points->points[j].x;
      if (dist > 0) {
        point_count++;
        break;
      }
    }
    if (point_count > 200) {
      valid_room_config = true;
      break;
    }
  }
  // std::cout<<"align check..."<<"point_count "<<point_count<<" size "<<size<<std::endl;
  return valid_room_config;
}

bool  is_x2_plane_aligned_w_y(
    const pcl::PointCloud<PointNormal>::Ptr x_plane2_points,
    const pcl::PointCloud<PointNormal>::Ptr y_plane_points) {
  bool valid_room_config = false;
  int point_count = 0;
  int size = y_plane_points->points.size();
  for (int i = 0; i < y_plane_points->points.size(); ++i) {
    for (int j = 0; j < x_plane2_points->points.size(); ++j) {
      float dist = y_plane_points->points[i].x - x_plane2_points->points[j].x;
      if (dist < 0) {
        point_count++;
        break;
      }
    }
    if (point_count > 200) {
      valid_room_config = true;
      break;
    }
  }
  // std::cout<<"align check..."<<"point_count "<<point_count<<" size "<<size<<std::endl;
  return valid_room_config;
}

bool  is_y1_plane_aligned_w_x(
    const pcl::PointCloud<PointNormal>::Ptr y_plane1_points,
    const pcl::PointCloud<PointNormal>::Ptr x_plane_points) {
  bool valid_room_config = false;
  int point_count = 0;
  int size = x_plane_points->points.size();
  for (int i = 0; i < x_plane_points->points.size(); ++i) {
    for (int j = 0; j < y_plane1_points->points.size(); ++j) {
      float dist = x_plane_points->points[i].y - y_plane1_points->points[j].y;
      if (dist > 0) {
        point_count++;
        break;
      }
    }
    if (point_count > 200) {
      valid_room_config = true;
      break;
    }
  }
  // std::cout<<"align check..."<<"point_count "<<point_count<<" size "<<size<<std::endl;
  return valid_room_config;
}

bool  is_y2_plane_aligned_w_x(
    const pcl::PointCloud<PointNormal>::Ptr y_plane2_points,
    const pcl::PointCloud<PointNormal>::Ptr x_plane_points) {
  bool valid_room_config = false;
  int point_count = 0;
  int size = x_plane_points->points.size();
  for (int i = 0; i < x_plane_points->points.size(); ++i) {
    for (int j = 0; j < y_plane2_points->points.size(); ++j) {
      float dist = x_plane_points->points[i].y - y_plane2_points->points[j].y;
      if (dist < 0) {
        point_count++;
        break;
      }
    }
    if (point_count > 200) {
      valid_room_config = true;
      break;
    }
  }
  // std::cout<<"align check..."<<"point_count "<<point_count<<" size "<<size<<std::endl;
  return valid_room_config;
}

std::vector<const s_graphs::VerticalPlanes*> obtain_planes_from_room(
    const s_graphs::Rooms& room,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& y_vert_planes) {
  std::vector<const s_graphs::VerticalPlanes*> planes;
  std::array<int, 4> ids = {room.plane_x1_id,
                            room.plane_x2_id,
                            room.plane_y1_id,
                            room.plane_y2_id};  // first index x_planes, second y_planes

  for (auto& id : ids) {
    // lambda funtion for finding a concrete id
    auto funct = [id](const s_graphs::VerticalPlanes& vectical_plane) {
      return id == vectical_plane.id;
    };
    // search over x_vert_planes
    if (planes.size() < 2) {
      auto it_x = x_vert_planes.find(id);
      if (it_x != x_vert_planes.end()) {
        planes.emplace_back(&(it_x->second));
      }
    } else {
      // search over y_vert_planes
      auto it_y = y_vert_planes.find(id);
      if (it_y != y_vert_planes.end()) {
        planes.emplace_back(&(it_y->second));
      }
    }
  }
  return planes;
}
std::vector<const s_graphs::VerticalPlanes*> obtain_planes_from_infinite_room(
    const s_graphs::InfiniteRooms& room,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& vert_planes) {
  std::vector<const s_graphs::VerticalPlanes*> planes;
  std::array<int, 2> ids = {room.plane1_id,
                            room.plane2_id};  
  for (auto& id : ids) {
    // lambda funtion for finding a concrete id
    auto funct = [id](const s_graphs::VerticalPlanes& vectical_plane) {
      return id == vectical_plane.id;
    };
    // search over vert_planes
    auto it_x = vert_planes.find(id);
    if (it_x != vert_planes.end()) {
      planes.emplace_back(&(it_x->second));
    }
  }
  return planes;
}


bool is_SE3_inside_a_room(const Eigen::Isometry3d& pose,
                          const std::vector<PlaneGlobalRep>& planes) {
  const Eigen::Vector3d point = pose.translation();
  // check if the point is in front of all the planes
  for (auto& plane : planes) {
    const Eigen::Vector3d plane_point = plane.point;
    auto diff_point = point - plane_point;

    if (plane.normal.dot(diff_point) < 0) {
      return false;
    }
  }
  return true;
}

std::optional<Eigen::Vector3d> find_intersection(const Eigen::Vector3d& point1,
                                                 const Eigen::Vector3d& direction1,
                                                 const Eigen::Vector3d& point2,
                                                 const Eigen::Vector3d& direction2) {
  // Calculate the cross product of the two direction vectors
  Eigen::Vector3d cross_product = direction1.cross(direction2);

  Eigen::Matrix2f A;
  A << direction1(0), -direction1(1), direction2(0), -direction2(1);
  Eigen::Vector2f b;
  b << point2(0) - point1(0), point2(1) - point1(1);
  Eigen::Vector2f x = A.colPivHouseholderQr().solve(b);
  Eigen::Vector3d intersection_point;
  intersection_point << point1(0) + x(0) * direction1(0),
      point1(1) + x(0) * direction1(1), point1(2) + x(0) * direction1(2);

  //   // If the cross product is zero, the lines are parallel and do not intersect
  // if (cross_product.isZero()) {
  //   return std::nullopt;
  // }

  // // Calculate the parameter values for each line
  // double t1 = (cross_product.dot(direction2.cross(point1 - point2))) /
  //             cross_product.dot(direction1);
  // // double t2 = (direction1.cross(point1 - point2)).dot(cross_product) /
  // //             cross_product.dot(direction1);

  // // Calculate the intersection point
  // Eigen::Vector3d intersection_point = point1 + direction1 * t1;

  return intersection_point;
}

std::optional<Eigen::Isometry3d> obtain_global_centre_of_room(
    const std::vector<PlaneGlobalRep>& planes) {
  Eigen::Isometry3d room_centre;
  room_centre = room_centre.Identity();

  // We consider that the planes are related 2 by 2 (2 in x ,and the other 2 in y)
  // x procedure:
  auto p1 = planes[0].point, p2 = planes[1].point;
  auto p1_normal = planes[0].normal;
  auto vec_p2_p1 = p2 - p1;
  Eigen::Vector3d mid_point_x_vec_respect_p1 =
      p1 + p1_normal * (planes[0].normal.dot(vec_p2_p1) * 0.5);

  auto p3 = planes[2].point, p4 = planes[3].point;
  auto p3_normal = planes[2].normal;
  auto vec_p4_p3 = p4 - p3;
  Eigen::Vector3d mid_point_y_vec_respect_p3 =
      p3 + p3_normal * (planes[2].normal.dot(vec_p4_p3) * 0.5);

  Eigen::Matrix3d rot = room_centre.rotation();
  rot.col(0) = p1_normal;
  rot.col(1) = p3_normal;
  rot.col(2) << p1_normal.cross(p3_normal);
  room_centre.linear() = rot;

  auto centre = find_intersection(
      mid_point_x_vec_respect_p1, p3_normal, mid_point_y_vec_respect_p3, p1_normal);
  if (!centre.has_value()) {
    return std::nullopt;
  }
  room_centre.translation() = centre.value();
  return room_centre;
}

std::vector<PlaneGlobalRep> obtain_global_planes_from_room(
    const s_graphs::Rooms& room,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& y_vert_planes) {
  std::vector<PlaneGlobalRep> plane_reps;
  auto planes = obtain_planes_from_room(room, x_vert_planes, y_vert_planes);
  if (planes.size() != 4) return {};
  // TODO:  DEBUG CAREFULLY
  for (auto& plane : planes) {
    PlaneGlobalRep plane_rep;
    plane_rep.normal = plane->plane.normal();
    Eigen::Vector3d glob_point = plane->plane.normal() * plane->plane.distance();
    plane_rep.point = glob_point;
    plane_reps.emplace_back(plane_rep);
  }
  return plane_reps;
}
std::vector<PlaneGlobalRep> obtain_global_planes_from_infinite_room(
    const s_graphs::InfiniteRooms& room,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& vert_planes) {
  std::vector<PlaneGlobalRep> plane_reps;
  auto planes = obtain_planes_from_infinite_room(room, vert_planes);
  if (planes.size() != 2) return {};
  // TODO:  DEBUG get_room_keyframes
  for (auto& plane : planes) {
    PlaneGlobalRep plane_rep;
    plane_rep.normal = plane->plane.normal();
    Eigen::Vector3d glob_point = plane->plane.normal() * plane->plane.distance();
    plane_rep.point = glob_point;
    plane_reps.emplace_back(plane_rep);
  }
  return plane_reps;
}

std::set<std::pair<int, g2o::VertexSE3*>> obtain_keyframe_candidates_from_room(
    const s_graphs::Rooms& room,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& y_vert_planes) {
  std::set<std::pair<int, g2o::VertexSE3*>> keyframe_candidates;
  auto planes = obtain_planes_from_room(room, x_vert_planes, y_vert_planes);
  if (planes.size() != 4) return {};
  for (auto& plane : planes) {
    for (auto& keyframe : plane->keyframe_node_vec) {
      keyframe_candidates.insert({keyframe->id(), keyframe});
    }
  }
  return keyframe_candidates;
}
std::set<std::pair<int, g2o::VertexSE3*>> obtain_keyframe_candidates_from_infinite_room(
    const s_graphs::InfiniteRooms& room,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& vert_planes) {
  std::set<std::pair<int, g2o::VertexSE3*>> keyframe_candidates;
  auto planes = obtain_planes_from_infinite_room(room, vert_planes);
  if (planes.size() != 2) return {};
  for (auto& plane : planes) {
    for (auto& keyframe : plane->keyframe_node_vec) {
      keyframe_candidates.insert({keyframe->id(), keyframe});
    }
  }
  return keyframe_candidates;
}

std::set<int> filter_keyframes_ids(
    const std::set<std::pair<int, g2o::VertexSE3*>>& candidate_keyframes,
    const std::vector<PlaneGlobalRep>& plane_reps) {
  std::set<int> keyframes_inside;

  for (auto& [id, keyframe] : candidate_keyframes) {
    if (!is_SE3_inside_a_room(keyframe->estimate(), plane_reps)) {
      continue;
    }
    keyframes_inside.insert({id});
  }

  return keyframes_inside;
}

std::set<g2o::VertexSE3*> publish_room_keyframes_ids(
    const s_graphs::Rooms& room,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& y_vert_planes) {
  std::set<g2o::VertexSE3*> keyframe_candidates;
  std::vector<PlaneGlobalRep> plane_reps;
  auto planes = obtain_planes_from_room(room, x_vert_planes, y_vert_planes);
  if (planes.size() != 4) return {};
  // TODO:  DEBUG CAREFULLY
  for (auto& plane : planes) {
    PlaneGlobalRep plane_rep;
    plane_rep.normal =
        plane->keyframe_node->estimate().linear() * plane->plane.normal();
    const auto glob_point = plane->keyframe_node->estimate() *
                            (-plane->plane.normal() * plane->plane.distance());
    plane_rep.point = glob_point;
    plane_reps.emplace_back(plane_rep);

    keyframe_candidates.insert(plane->keyframe_node_vec.begin(),
                               plane->keyframe_node_vec.end());
  }
  std::set<g2o::VertexSE3*> keyframes_inside;

  for (auto& keyframe_candidate : keyframe_candidates) {
    if (!is_SE3_inside_a_room(keyframe_candidate->estimate(), plane_reps)) {
      continue;
    }
    keyframes_inside.insert(keyframe_candidate);
  }

  return keyframes_inside;
}

std::set<g2o::VertexSE3*> filter_inside_room_keyframes(
    const s_graphs::Rooms& room,
    const std::set<g2o::VertexSE3*>& keyframes_candidates) {
  std::set<g2o::VertexSE3*> final_candidates;
  // room.node->estimate();

  return final_candidates;
}

template <typename KeyFramePtrVec>
KeyFramePtrVec obtain_keyframes_from_ids(const std::set<int>& id_list,
                                         const KeyFramePtrVec& _keyframes) {
  KeyFramePtrVec keyframes;
  for (auto& id : id_list) {
    auto it = _keyframes.find(id);
    if (it == _keyframes.end()) {
      continue;
    }
    keyframes.insert({id, it->second});
  }
  return keyframes;
}

template <typename KeyFramePtrVec> //std::map<int, s_graphs::KeyFrame::Ptr>
void obtain_keyframes_from_ids(const std::set<int>& id_list,
                              const KeyFramePtrVec& _keyframes,
                              const string & room_type,
                              const double& room_start, const double& room_end,
                              const double& roompart_length,
                              KeyFramePtrVec& keyframes,
                              std::map<int, std::tuple<double, double, KeyFramePtrVec>>& new_roompart_keyframes) {
  int part_num = 0; 
  double mid_x = 0; double mid_y = 0;
  new_roompart_keyframes.clear();
  double tmp=room_start;
  while (true){
    // std::cout<<"tmp "<<tmp<<" "<<room_start<<" "<<room_end<<std::endl;
    if (tmp >= room_end) break;

    if (room_type=="x") mid_y = tmp+0.5*roompart_length;
    if (room_type=="y") mid_x = tmp+0.5*roompart_length;
    new_roompart_keyframes.insert({part_num, {mid_x, mid_y, {}}});
    part_num++;
    tmp += roompart_length;
  }

  for (auto& id : id_list) {
    auto it = _keyframes.find(id);
    if (it == _keyframes.end()) {
      continue;
    }
    keyframes.insert({id, it->second});
    for (auto& kf : keyframes) {
      double kf_tmp;
      if (room_type=="x") kf_tmp = kf.second->odom.translation().y();
      else kf_tmp = kf.second->odom.translation().x();
      if (kf_tmp<room_start) continue;

      int part_num_tmp = (int)((kf_tmp-room_start)/roompart_length);
      if (part_num_tmp>=0 && part_num_tmp<part_num)
        std::get<2>(new_roompart_keyframes[part_num_tmp]).insert(kf);
    }
  }

  return ;
}

std::optional<
    std::pair<Eigen::Isometry3d, pcl::PointCloud<s_graphs::KeyFrame::PointT>::Ptr>>
generate_room_keyframe(
    const s_graphs::Rooms& room,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& y_vert_planes,
    const std::map<int, s_graphs::KeyFrame::Ptr>& keyframes) {
  auto global_planes =
      obtain_global_planes_from_room(room, x_vert_planes, y_vert_planes);
  auto room_centre = obtain_global_centre_of_room(global_planes);
  // If room centre coudn't be computed return
  if (!room_centre.has_value()) return {};

  auto keyframe_candidates =
      obtain_keyframe_candidates_from_room(room, x_vert_planes, y_vert_planes);
  auto keyframes_ids = filter_keyframes_ids(keyframe_candidates, global_planes);
  auto keyframes_vec = obtain_keyframes_from_ids(keyframes_ids, keyframes);
  auto cloud = generate_room_pointcloud(
      room, room_centre.value(), keyframes_vec.begin(), keyframes_vec.end());
  return {{room_centre.value(), cloud}};
  // room_centre.value(), cloud)};
}

std::map<int, s_graphs::KeyFrame::Ptr> get_room_keyframes(
    const s_graphs::Rooms& room,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& y_vert_planes,
    const std::map<int, s_graphs::KeyFrame::Ptr>& keyframes) {
  auto global_planes =
      obtain_global_planes_from_room(room, x_vert_planes, y_vert_planes);
  auto room_centre = obtain_global_centre_of_room(global_planes);

  auto keyframe_candidates =
      obtain_keyframe_candidates_from_room(room, x_vert_planes, y_vert_planes);

  auto keyframes_ids = filter_keyframes_ids(keyframe_candidates, global_planes);

  auto keyframes_vec = obtain_keyframes_from_ids(keyframes_ids, keyframes);

  return keyframes_vec;
}

std::map<int, s_graphs::KeyFrame::Ptr> get_infinite_room_keyframes(
     s_graphs::InfiniteRooms& room,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& vert_planes,
    const std::map<int, s_graphs::KeyFrame::Ptr>& keyframes) {
  auto global_planes =
      obtain_global_planes_from_infinite_room(room, vert_planes);
  if (global_planes.empty()) return {};

  auto keyframe_candidates =
      obtain_keyframe_candidates_from_infinite_room(room, vert_planes);
  if (keyframe_candidates.empty()) return {};

  auto keyframes_ids = filter_keyframes_ids(keyframe_candidates, global_planes);

  auto keyframes_vec = obtain_keyframes_from_ids(keyframes_ids, keyframes);

  return keyframes_vec;
}
std::map<int, s_graphs::KeyFrame::Ptr> get_infinite_room_keyframes(
    const s_graphs::InfiniteRooms& room,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& vert_planes,
    const std::map<int, s_graphs::KeyFrame::Ptr>& keyframes) {
  auto global_planes =
      obtain_global_planes_from_infinite_room(room, vert_planes);
  if (global_planes.empty()) return {};

  auto keyframe_candidates =
      obtain_keyframe_candidates_from_infinite_room(room, vert_planes);
  if (keyframe_candidates.empty()) return {};

  auto keyframes_ids = filter_keyframes_ids(keyframe_candidates, global_planes);

  auto keyframes_vec = obtain_keyframes_from_ids(keyframes_ids, keyframes);

  return keyframes_vec;
}

void get_room_keyframes(
    s_graphs::Rooms& room,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& y_vert_planes,
    const std::map<int, s_graphs::KeyFrame::Ptr>& keyframes,
          const std::string & room_type,
          const double& room_start,
          const double& room_end,
          const double& roompart_length) {
  auto global_planes =
      obtain_global_planes_from_room(room, x_vert_planes, y_vert_planes);
  auto room_centre = obtain_global_centre_of_room(global_planes);

  auto keyframe_candidates =
      obtain_keyframe_candidates_from_room(room, x_vert_planes, y_vert_planes);

  auto keyframes_ids = filter_keyframes_ids(keyframe_candidates, global_planes);

  obtain_keyframes_from_ids(keyframes_ids, keyframes, room_type, room_start, room_end,
                          roompart_length, room.room_keyframes, room.new_roompart_keyframes);
  return;
}
void get_infinite_room_keyframes(
    s_graphs::InfiniteRooms& room,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& vert_planes,
    const std::map<int, s_graphs::KeyFrame::Ptr>& keyframes,
          const std::string & room_type,
          const double& room_start,
          const double& room_end,
          const double& roompart_length) {
  auto global_planes =
      obtain_global_planes_from_infinite_room(room, vert_planes);
  if (global_planes.empty()) return ;

  auto keyframe_candidates =
      obtain_keyframe_candidates_from_infinite_room(room, vert_planes);
  if (keyframe_candidates.empty()) return ;

  auto keyframes_ids = filter_keyframes_ids(keyframe_candidates, global_planes);

  obtain_keyframes_from_ids(keyframes_ids, keyframes, room_type, room_start, room_end,
                          roompart_length, room.room_keyframes, room.new_roompart_keyframes);
  return;
}

bool is_keyframe_inside_room(
    const s_graphs::Rooms& room,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& y_vert_planes,
    const s_graphs::KeyFrame::Ptr keyframe) {
  auto global_planes =
      obtain_global_planes_from_room(room, x_vert_planes, y_vert_planes);

  if (is_SE3_inside_a_room(keyframe->estimate(), global_planes)) {
    return true;
  }

  return false;
}
bool is_keyframe_inside_infinite_room(
    const s_graphs::InfiniteRooms& room,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& vert_planes,
    const s_graphs::KeyFrame::Ptr keyframe) {
  auto global_planes =
      obtain_global_planes_from_infinite_room(room, vert_planes);
  if (global_planes.empty()) return false;
  if (is_SE3_inside_a_room(keyframe->estimate(), global_planes)) {
    return true;
  }
  return false;
}

situational_graphs_reasoning_msgs::msg::RoomKeyframe convertExtendedRoomToRosMsg(
    const ExtendedRooms& room) {
  situational_graphs_reasoning_msgs::msg::RoomKeyframe msg;
  msg.header.frame_id = "room";
  msg.id = room.id;
  pcl::toROSMsg(*room.cloud, msg.pointcloud);
  msg.pose = s_graphs::isometry2pose(room.centre);
  for (auto& keyframe : room.keyframes) {
    msg.keyframes_ids.emplace_back(keyframe->id());
  }
  msg.planes_ids.emplace_back(room.plane_x1_id);
  msg.planes_ids.emplace_back(room.plane_x2_id);
  msg.planes_ids.emplace_back(room.plane_y1_id);
  msg.planes_ids.emplace_back(room.plane_y2_id);

  return msg;
}

ExtendedRooms obtainExtendedRoomFromRosMsg(
    const situational_graphs_reasoning_msgs::msg::RoomKeyframe& msg) {
  ExtendedRooms room;
  room.id = msg.id;
  pcl::fromROSMsg(msg.pointcloud, *room.cloud);
  room.centre = s_graphs::pose2isometry(msg.pose);
  if (msg.planes_ids.size() != 4) {
    std::cout << "room doesnt contain 4 planes" << std::endl;
    return room;
  }
  room.plane_x1_id = msg.planes_ids[0];
  room.plane_x2_id = msg.planes_ids[1];
  room.plane_y1_id = msg.planes_ids[2];
  room.plane_y2_id = msg.planes_ids[3];

  // TODO: Fill these fields
  /* for (auto& keyframe : msg.keyframes_ids) {
    room.keyframes
  }
  msg.planes_ids.emplace_back(room.plane_x1_id);
  msg.planes_ids.emplace_back(room.plane_x2_id);
  msg.planes_ids.emplace_back(room.plane_y1_id);
  msg.planes_ids.emplace_back(room.plane_y2_id); */

  return room;
}
