#include <s_graphs/backend/room_mapper.hpp>

namespace s_graphs {

InfiniteRoomMapper::InfiniteRoomMapper(const rclcpp::Node::SharedPtr node) {
  node_obj = node;

  infinite_room_information = node->get_parameter("infinite_room_information")
                                  .get_parameter_value()
                                  .get<double>();
  infinite_room_dist_threshold = node->get_parameter("infinite_room_dist_threshold")
                                     .get_parameter_value()
                                     .get<double>();
  room_dist_threshold = node->get_parameter("room_dist_threshold")
                                     .get_parameter_value()
                                     .get<double>();
  dupl_plane_matching_information =
      node->get_parameter("dupl_plane_matching_information")
          .get_parameter_value()
          .get<double>();

  use_parallel_plane_constraint = node->get_parameter("use_parallel_plane_constraint")
                                      .get_parameter_value()
                                      .get<bool>();
  use_perpendicular_plane_constraint =
      node->get_parameter("use_perpendicular_plane_constraint")
          .get_parameter_value()
          .get<bool>();
  keyframe_window_size = node->get_parameter("keyframe_window_size")
                                      .get_parameter_value().get<int>();

  roompart_length =
      node->get_parameter("roompart_length")
          .get_parameter_value().get<double>();    
}

InfiniteRoomMapper::~InfiniteRoomMapper() {}

bool InfiniteRoomMapper::lookup_infinite_rooms(
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
    bool& matched) {
  Eigen::Isometry3d room_center;
  Eigen::Quaterniond room_quat;
  bool duplicate_found = false;

  room_quat.x() = room_data.room_center.orientation.x;
  room_quat.y() = room_data.room_center.orientation.y;
  room_quat.z() = room_data.room_center.orientation.z;
  room_quat.w() = room_data.room_center.orientation.w;
  room_center.linear() = room_quat.toRotationMatrix();
  room_center.translation().x() = room_data.room_center.position.x;
  room_center.translation().y() = room_data.room_center.position.y;
  room_center.translation().z() = room_data.room_center.position.z;

  Eigen::Isometry3d cluster_center;
  Eigen::Quaterniond cluster_quat;
  cluster_quat.x() = 0;
  cluster_quat.y() = 0;
  cluster_quat.z() = 0;
  cluster_quat.w() = 1;
  cluster_center.linear() = cluster_quat.toRotationMatrix();
  cluster_center.translation().x() = room_data.cluster_center.x;
  cluster_center.translation().y() = room_data.cluster_center.y;
  cluster_center.translation().z() = room_data.cluster_center.z;

  if (plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
    // factor the infinite_room here
    std::cout << "factoring x infinite_room" << std::endl;

    Eigen::Vector4d x_plane1(room_data.x_planes[0].nx,
                             room_data.x_planes[0].ny,
                             room_data.x_planes[0].nz,
                             room_data.x_planes[0].d);
    Eigen::Vector4d x_plane2(room_data.x_planes[1].nx,
                             room_data.x_planes[1].ny,
                             room_data.x_planes[1].nz,
                             room_data.x_planes[1].d);
    plane_data_list x_plane1_data, x_plane2_data;
    x_plane1_data.plane_id = room_data.x_planes[0].id;
    x_plane1_data.plane_unflipped = x_plane1;
    x_plane2_data.plane_id = room_data.x_planes[1].id;
    x_plane2_data.plane_unflipped = x_plane2;

    duplicate_found = factor_infinite_rooms(graph_slam,
                                            PlaneUtils::plane_class::X_VERT_PLANE,
                                            x_plane1_data,
                                            x_plane2_data,
                                            x_vert_planes,
                                            y_vert_planes,
                                            dupl_x_vert_planes,
                                            dupl_y_vert_planes,
                                            x_infinite_rooms,
                                            y_infinite_rooms,
                                            rooms_vec,
                                            room_id,
                                            room_center,
                                            room_data.room_length,
                                            cluster_center,
                                            room_data.cluster_array,
                                            keyframes,
                                            matched);
  }

  else if (plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
    // factor the infinite_room here
    std::cout << "factoring y infinite_room" << std::endl;
    Eigen::Vector4d y_plane1(room_data.y_planes[0].nx,
                             room_data.y_planes[0].ny,
                             room_data.y_planes[0].nz,
                             room_data.y_planes[0].d);
    Eigen::Vector4d y_plane2(room_data.y_planes[1].nx,
                             room_data.y_planes[1].ny,
                             room_data.y_planes[1].nz,
                             room_data.y_planes[1].d);
    plane_data_list y_plane1_data, y_plane2_data;
    y_plane1_data.plane_id = room_data.y_planes[0].id;
    y_plane1_data.plane_unflipped = y_plane1;
    y_plane2_data.plane_id = room_data.y_planes[1].id;
    y_plane2_data.plane_unflipped = y_plane2;

    duplicate_found = factor_infinite_rooms(graph_slam,
                                            PlaneUtils::plane_class::Y_VERT_PLANE,
                                            y_plane1_data,
                                            y_plane2_data,
                                            x_vert_planes,
                                            y_vert_planes,
                                            dupl_x_vert_planes,
                                            dupl_y_vert_planes,
                                            x_infinite_rooms,
                                            y_infinite_rooms,
                                            rooms_vec,
                                            room_id,
                                            room_center,
                                            room_data.room_length,
                                            cluster_center,
                                            room_data.cluster_array,
                                            keyframes,
                                            matched);
  }

  return duplicate_found;
}

bool InfiniteRoomMapper::factor_infinite_rooms(
    std::shared_ptr<GraphSLAM>& graph_slam,
    const int plane_type,
    const plane_data_list& room_plane1_pair,
    const plane_data_list& room_plane2_pair,
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
    bool& matched) {
  g2o::VertexRoom* room_node;
  g2o::VertexRoom* cluster_center_node;
  int room_data_association = -1;
  int finite_room_data_association = -1;
  bool duplicate_found = false;

  Eigen::Matrix<double, 2, 2> information_infinite_room_planes;
  information_infinite_room_planes.setZero();
  information_infinite_room_planes(0, 0) = infinite_room_information;
  information_infinite_room_planes(1, 1) = infinite_room_information;

  Eigen::Matrix<double, 1, 1> information_infinite_room_plane;
  information_infinite_room_plane(0, 0) = infinite_room_information;

  Eigen::Matrix<double, 1, 1> information_infinite_room_prior;
  information_infinite_room_prior(0, 0) = 1e-5;

  Eigen::Matrix<double, 3, 3> information_2planes;
  information_2planes.setZero();
  information_2planes(0, 0) = dupl_plane_matching_information;
  information_2planes(1, 1) = dupl_plane_matching_information;
  information_2planes(2, 2) = dupl_plane_matching_information;

  Eigen::Matrix<double, 4, 4> information_planes_identity;
  information_planes_identity.setZero();
  information_planes_identity(0, 0) = dupl_plane_matching_information;
  information_planes_identity(1, 1) = dupl_plane_matching_information;
  information_planes_identity(2, 2) = dupl_plane_matching_information;
  information_planes_identity(3, 3) = dupl_plane_matching_information;
  RCLCPP_DEBUG(node_obj->get_logger(),
               "infinite_room planes",
               "final infinite_room plane 1 %f %f %f %f",
               room_plane1_pair.plane_unflipped.coeffs()(0),
               room_plane1_pair.plane_unflipped.coeffs()(1),
               room_plane1_pair.plane_unflipped.coeffs()(2),
               room_plane1_pair.plane_unflipped.coeffs()(3));
  RCLCPP_DEBUG(node_obj->get_logger(),
               "infinite_room planes",
               "final infinite_room plane 2 %f %f %f %f",
               room_plane2_pair.plane_unflipped.coeffs()(0),
               room_plane2_pair.plane_unflipped.coeffs()(1),
               room_plane2_pair.plane_unflipped.coeffs()(2),
               room_plane2_pair.plane_unflipped.coeffs()(3));

  if (plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
    auto found_plane1 = x_vert_planes.find(room_plane1_pair.plane_id);
    auto found_plane2 = x_vert_planes.find(room_plane2_pair.plane_id);

    if (found_plane1 == x_vert_planes.end() || found_plane2 == x_vert_planes.end()) {
      std::cout << "did not find planes for x infinite_room " << std::endl;
      return duplicate_found;
    }

    std::vector<std::pair<VerticalPlanes, VerticalPlanes>> detected_mapped_plane_pairs;
    finite_room_data_association = associate_rooms(plane_type,
                                            room_center,
                                            room_length,
                                            cluster_array,
                                            (found_plane1->second),
                                            (found_plane2->second),
                                            x_vert_planes,
                                            y_vert_planes,
                                            rooms_vec,
                                            detected_mapped_plane_pairs);
  std::cout<<"finite_room_data_association "<<finite_room_data_association<<std::endl;
  if (finite_room_data_association!=-1){
      /* add the edge between detected planes and the infinite_room */
      room_node = rooms_vec[finite_room_data_association].node;
      room_id = finite_room_data_association;
      std::cout << "Matched det infinite_room X with pre pose "
                << room_center.translation() << " to mapped finite_room with id "
                << finite_room_data_association << " and pose "
                << room_node->estimate().translation() << std::endl;
      matched = true;
      
      InfiniteRooms det_infinite_room;
      det_infinite_room.id = finite_room_data_association;
      det_infinite_room.plane1 = room_plane1_pair.plane_unflipped;
      det_infinite_room.plane2 = room_plane2_pair.plane_unflipped;
      det_infinite_room.plane1_id = room_plane1_pair.plane_id;
      det_infinite_room.plane2_id = room_plane2_pair.plane_id;

      det_infinite_room.room_max = room_center.translation().y()+room_length.y/2;
      det_infinite_room.room_min = room_center.translation().y()-room_length.y/2;
      rooms_vec[finite_room_data_association].room_y_max = max(rooms_vec[finite_room_data_association].room_y_max, det_infinite_room.room_max );
      rooms_vec[finite_room_data_association].room_y_min = min(rooms_vec[finite_room_data_association].room_y_min, det_infinite_room.room_min );

      for (auto &kf: rooms_vec[finite_room_data_association].room_keyframes) {
        std::get<2>(rooms_vec[finite_room_data_association].old_roompart_keyframes[0]).insert(kf); }
      rooms_vec[finite_room_data_association].old_roompart_keyframes.insert(
                {0,{0,0,rooms_vec[finite_room_data_association].room_keyframes}});
      det_infinite_room.room_keyframes = get_infinite_room_keyframes(det_infinite_room, x_vert_planes, keyframes);
      rooms_vec[finite_room_data_association].new_roompart_keyframes.clear();
      rooms_vec[finite_room_data_association].new_roompart_keyframes.insert(
        {0,{0,0,det_infinite_room.room_keyframes}});

      mergeKeyframes(rooms_vec[finite_room_data_association].room_keyframes, det_infinite_room.room_keyframes);

      // mergeMarkerArrays(rooms_vec[finite_room_data_association], cluster_array);
      // setRoomCenter(rooms_vec[finite_room_data_association], room_center);
      setLargerPlanes(rooms_vec[finite_room_data_association], "x", det_infinite_room);
      
      return duplicate_found;

    }
    // if (finite_room_data_association==-1) {
    else {
      detected_mapped_plane_pairs.clear();
      room_data_association = associate_infinite_rooms(graph_slam,
                                          plane_type,
                                          room_center,
                                          room_length,
                                          cluster_array,
                                          (found_plane1->second),
                                          (found_plane2->second),
                                          x_vert_planes,
                                          y_vert_planes,
                                          x_infinite_rooms,
                                          y_infinite_rooms,
                                          detected_mapped_plane_pairs);
    }
    std::cout<<"room_data_association "<<room_data_association<<std::endl;

    if ( (x_infinite_rooms.empty() && room_data_association!=-1 && y_infinite_rooms.find(room_data_association)==y_infinite_rooms.end() )
            || (finite_room_data_association == -1 && room_data_association == -1)) {
      std::cout << "found an X infinite_room with pre pose "
                << room_center.translation() << " between plane "
                << room_plane1_pair.plane_unflipped.coeffs() << " and plane "
                << room_plane2_pair.plane_unflipped.coeffs() << std::endl;

      room_data_association = graph_slam->retrieve_local_nbr_of_vertices();
      room_node = graph_slam->add_room_node(room_center);
      // cluster_center_node = graph_slam->add_room_node(cluster_center);
      // cluster_center_node->setFixed(true);

      InfiniteRooms det_infinite_room;
      det_infinite_room.id = room_data_association;
      room_id = det_infinite_room.id;
      det_infinite_room.plane1 = room_plane1_pair.plane_unflipped;
      det_infinite_room.plane2 = room_plane2_pair.plane_unflipped;
      det_infinite_room.plane1_id = room_plane1_pair.plane_id;
      det_infinite_room.plane2_id = room_plane2_pair.plane_id;
      det_infinite_room.cluster_array = cluster_array;

      det_infinite_room.room_max = room_center.translation().y()+room_length.y/2;
      det_infinite_room.room_min = room_center.translation().y()-room_length.y/2;
      // add keyframe
      // det_infinite_room.room_keyframes = get_infinite_room_keyframes(det_infinite_room, x_vert_planes, keyframes);
      det_infinite_room.room_keyframes = get_infinite_room_keyframes(det_infinite_room, x_vert_planes, keyframes);
      
      det_infinite_room.old_roompart_keyframes.clear();
      det_infinite_room.new_roompart_keyframes.clear();
      det_infinite_room.new_roompart_keyframes.insert({0,{0,0,det_infinite_room.room_keyframes}});

      // det_infinite_room.cluster_center_node = cluster_center_node;
      det_infinite_room.node = room_node;
      det_infinite_room.plane1_node = (found_plane1->second).plane_node;
      det_infinite_room.plane2_node = (found_plane2->second).plane_node;
      det_infinite_room.local_graph = std::make_shared<GraphSLAM>();
      x_infinite_rooms.insert({det_infinite_room.id, det_infinite_room});

      auto edge_room_plane =
          graph_slam->add_room_2planes_edge(room_node,
                                            (found_plane1->second).plane_node,
                                            (found_plane2->second).plane_node,
                                            room_node,
                                            information_infinite_room_planes);
      graph_slam->add_robust_kernel(edge_room_plane, "Huber", 1.0);

    } 
    else {
      std::cout<<"else"<<std::endl;
      
      if (x_infinite_rooms.find(room_data_association)!= x_infinite_rooms.end()) {
        /* add the edge between detected planes and the infinite_room */
        room_node = x_infinite_rooms[room_data_association].node;
        room_id = room_data_association;
        std::cout << "Matched det infinite_room X with pre pose "
                  << room_center.translation() << " to mapped infinite_room with id "
                  << room_data_association << " and pose "
                  << room_node->estimate().translation() << std::endl;
        matched = true;
        
        update_old_roompart_keyframes(x_infinite_rooms[room_data_association].room_keyframes,
                        "x", x_infinite_rooms[room_data_association].room_min, x_infinite_rooms[room_data_association].room_max, roompart_length,
                        x_infinite_rooms[room_data_association].old_roompart_keyframes);

        InfiniteRooms det_infinite_room;
        det_infinite_room.id = room_data_association;
        det_infinite_room.plane1 = room_plane1_pair.plane_unflipped;
        det_infinite_room.plane2 = room_plane2_pair.plane_unflipped;
        det_infinite_room.plane1_id = room_plane1_pair.plane_id;
        det_infinite_room.plane2_id = room_plane2_pair.plane_id;
        
        det_infinite_room.room_max = room_center.translation().y()+room_length.y/2;
        det_infinite_room.room_min = room_center.translation().y()-room_length.y/2;
        
        // for (auto &kf: x_infinite_rooms[room_data_association].room_keyframes) {
        //   std::get<2>(x_infinite_rooms[room_data_association].old_roompart_keyframes[0]).insert(kf); }
        det_infinite_room.room_keyframes = get_infinite_room_keyframes(det_infinite_room, x_vert_planes, keyframes);
        update_old_roompart_keyframes(det_infinite_room.room_keyframes,
                        "x", det_infinite_room.room_min, det_infinite_room.room_max, roompart_length,
                        det_infinite_room.new_roompart_keyframes);

        x_infinite_rooms[room_data_association].room_max = max(x_infinite_rooms[room_data_association].room_max, det_infinite_room.room_max);
        x_infinite_rooms[room_data_association].room_min = min(x_infinite_rooms[room_data_association].room_min, det_infinite_room.room_min);
        // x_infinite_rooms[room_data_association].new_roompart_keyframes.clear();
        // x_infinite_rooms[room_data_association].new_roompart_keyframes.insert(
        //   {0,{0,0,det_infinite_room.room_keyframes}});
        mergeKeyframes(x_infinite_rooms[room_data_association].room_keyframes, det_infinite_room.room_keyframes);

        mergeMarkerArrays(x_infinite_rooms[room_data_association], "x", cluster_array);
        setRoomCenter(x_infinite_rooms[room_data_association], "x", room_center);
        setLargerPlanes(x_infinite_rooms[room_data_association], det_infinite_room);
        
        x_infinite_rooms[room_data_association].new_roompart_keyframes.clear();
        for (auto new_part: det_infinite_room.new_roompart_keyframes){
          x_infinite_rooms[room_data_association].new_roompart_keyframes.insert(new_part);
        }

      }
      else if (room_data_association!=-1 && y_infinite_rooms.find(room_data_association)!= y_infinite_rooms.end()) {
      std::cout<<"y_infinite_rooms"<<std::endl;
        // return duplicate_found; 
        InfiniteRooms det_infinite_room;
        det_infinite_room.plane1 = room_plane1_pair.plane_unflipped;
        det_infinite_room.plane2 = room_plane2_pair.plane_unflipped;
        det_infinite_room.plane1_id = room_plane1_pair.plane_id;
        det_infinite_room.plane2_id = room_plane2_pair.plane_id;
        
        det_infinite_room.room_max = room_center.translation().y()+room_length.y/2;
        det_infinite_room.room_min = room_center.translation().y()-room_length.y/2;
        det_infinite_room.room_keyframes = get_infinite_room_keyframes(det_infinite_room, x_vert_planes, keyframes);
        
        
        room_id = room_data_association;
        std::cout << "Matched det infinite_room Y with pre pose "
                  << room_center.translation() << " to mapped finite_room with id "
                  << room_data_association << " and pose "
                  << room_node->estimate().translation() << std::endl;
        matched = true;

        Rooms det_room;
        det_room.id = room_id;

        // mergeMarkerArrays(y_infinite_rooms[room_data_association].cluster_array, "y", cluster_array);
        Eigen::Isometry3d updated_room_center = Eigen::Isometry3d::Identity();
        updated_room_center.translation().x() = 0.5*(y_infinite_rooms[room_data_association].room_max+y_infinite_rooms[room_data_association].room_min);
        updated_room_center.translation().y() = room_center.translation()(1);
        updated_room_center.linear() = room_center.rotation();

        room_node = graph_slam->add_room_node(updated_room_center);
        
        det_room.cluster_array = cluster_array;

        det_room.plane_x1 = det_infinite_room.plane1;
        det_room.plane_x2 = det_infinite_room.plane2;
        det_room.plane_y1 = y_infinite_rooms[room_data_association].plane1;
        det_room.plane_y2 = y_infinite_rooms[room_data_association].plane2;
        det_room.plane_x1_id = det_infinite_room.plane1_id;
        det_room.plane_x2_id = det_infinite_room.plane2_id;
        det_room.plane_y1_id = y_infinite_rooms[room_data_association].plane1_id;
        det_room.plane_y2_id = y_infinite_rooms[room_data_association].plane2_id;
        det_room.plane_x1_node = (found_plane1->second).plane_node;
        det_room.plane_x2_node = (found_plane2->second).plane_node;
        det_room.plane_y1_node = y_infinite_rooms[room_data_association].plane1_node;
        det_room.plane_y2_node = y_infinite_rooms[room_data_association].plane2_node;

        det_room.room_x_max = max(room_center.translation().x()+room_length.x/2, y_infinite_rooms[room_data_association].room_max);
        det_room.room_x_min = min(room_center.translation().x()-room_length.x/2, y_infinite_rooms[room_data_association].room_min);
        det_room.room_y_max = det_infinite_room.room_max;
        det_room.room_y_min = det_infinite_room.room_min;

        det_room.local_graph = std::make_shared<GraphSLAM>();
        det_room.node = room_node;
        rooms_vec.insert({det_room.id, det_room});
        
        for (auto &kf:  y_infinite_rooms[room_data_association].room_keyframes) {
          std::get<2>(det_room.old_roompart_keyframes[0]).insert(kf); }
        
        mergeKeyframes(det_room.room_keyframes, y_infinite_rooms[room_data_association].room_keyframes);
        mergeKeyframes(det_room.room_keyframes, det_infinite_room.room_keyframes);

        det_room.new_roompart_keyframes.insert({0,{0,0,det_infinite_room.room_keyframes}});
        
        mergeMarkerArrays(det_room, y_infinite_rooms[room_data_association].cluster_array);
        mergeMarkerArrays(det_room, det_infinite_room.cluster_array);
    
        remove_mapped_infinite_room(PlaneUtils::plane_class::Y_VERT_PLANE,
                                  graph_slam,
                                  y_infinite_rooms[room_data_association],
                                  x_infinite_rooms,
                                  y_infinite_rooms);
        
      }
      std::set<g2o::HyperGraph::Edge*> plane1_edges =
          (found_plane1->second).plane_node->edges();
      std::set<g2o::HyperGraph::Edge*> plane2_edges =
          (found_plane2->second).plane_node->edges();
 
      if (detected_mapped_plane_pairs[0].first.id != detected_mapped_plane_pairs[0].second.id) {
        if (!MapperUtils::check_plane_ids(
                plane1_edges, detected_mapped_plane_pairs[0].second.plane_node)
          && PlaneUtils::plane_dot_product(detected_mapped_plane_pairs[0].first.plane_node
                , detected_mapped_plane_pairs[0].second.plane_node)> 0.9) {
          auto edge_planes = graph_slam->add_2planes_edge(
              detected_mapped_plane_pairs[0].first.plane_node,
              detected_mapped_plane_pairs[0].second.plane_node,
              information_2planes);
          // Eigen::Vector4d mea(0,0,0,0);
          // auto edge_planes = graph_slam->add_plane_identity_edge(
          //     detected_mapped_plane_pairs[0].first.plane_node,
          //     detected_mapped_plane_pairs[0].second.plane_node,
          //     mea,
          //     information_planes_identity);
          graph_slam->add_robust_kernel(edge_planes, "Huber", 1.0);
          duplicate_found = true;
          std::cout << "Adding new x1 plane " << std::endl;
        }
      }

      if (detected_mapped_plane_pairs[1].first.id != detected_mapped_plane_pairs[1].second.id) {
        if (!MapperUtils::check_plane_ids(
                plane2_edges, detected_mapped_plane_pairs[1].second.plane_node)
          && PlaneUtils::plane_dot_product(detected_mapped_plane_pairs[1].first.plane_node
                , detected_mapped_plane_pairs[1].second.plane_node)> 0.9) {
          auto edge_planes = graph_slam->add_2planes_edge(
              detected_mapped_plane_pairs[1].first.plane_node,
              detected_mapped_plane_pairs[1].second.plane_node,
              information_2planes);
          
          // Eigen::Vector4d mea(0,0,0,0);
          // auto edge_planes = graph_slam->add_plane_identity_edge(
          //     detected_mapped_plane_pairs[1].first.plane_node,
          //     detected_mapped_plane_pairs[1].second.plane_node,
          //     mea,
          //     information_planes_identity);
          graph_slam->add_robust_kernel(edge_planes, "Huber", 1.0);
          duplicate_found = true;
          std::cout << "Adding new x2 plane " << std::endl;
        }
      }
    }
  }

  if (plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
    auto found_plane1 = y_vert_planes.find(room_plane1_pair.plane_id);
    auto found_plane2 = y_vert_planes.find(room_plane2_pair.plane_id);

    if (found_plane1 == y_vert_planes.end() || found_plane2 == y_vert_planes.end())
      return duplicate_found;

    std::vector<std::pair<VerticalPlanes, VerticalPlanes>> detected_mapped_plane_pairs;
    finite_room_data_association = associate_rooms(plane_type,
                                            room_center,
                                            room_length,
                                            cluster_array,
                                            (found_plane1->second),
                                            (found_plane2->second),
                                            x_vert_planes,
                                            y_vert_planes,
                                            rooms_vec,
                                            detected_mapped_plane_pairs);
    if (finite_room_data_association!=-1){
      /* add the edge between detected planes and the infinite_room */
      room_node = rooms_vec[finite_room_data_association].node;
      room_id = finite_room_data_association;
      std::cout << "Matched det infinite_room Y with pre pose "
                << room_center.translation() << " to mapped finite_room with id "
                << finite_room_data_association << " and pose "
                << room_node->estimate().translation() << std::endl;
      matched = true;

      InfiniteRooms det_infinite_room;
      // det_infinite_room.id = finite_room_data_association;
      det_infinite_room.plane1 = room_plane1_pair.plane_unflipped;
      det_infinite_room.plane2 = room_plane2_pair.plane_unflipped;
      det_infinite_room.plane1_id = room_plane1_pair.plane_id;
      det_infinite_room.plane2_id = room_plane2_pair.plane_id;
      
      // det_infinite_room.room_max = room_center.translation().x()+room_length.x/2;
      // det_infinite_room.room_min = room_center.translation().x()-room_length.x/2;
      // rooms_vec[finite_room_data_association].room_x_max = max(rooms_vec[finite_room_data_association].room_x_max, det_infinite_room.room_max );
      // rooms_vec[finite_room_data_association].room_x_min = min(rooms_vec[finite_room_data_association].room_x_min, det_infinite_room.room_min );
      
      for (auto &kf: rooms_vec[finite_room_data_association].room_keyframes) {
        std::get<2>(rooms_vec[finite_room_data_association].old_roompart_keyframes[0]).insert(kf); }
      det_infinite_room.room_keyframes = get_infinite_room_keyframes(det_infinite_room, y_vert_planes, keyframes);
      rooms_vec[finite_room_data_association].new_roompart_keyframes.clear();
      rooms_vec[finite_room_data_association].new_roompart_keyframes.insert( {0,{0,0,det_infinite_room.room_keyframes}});
      mergeKeyframes(rooms_vec[finite_room_data_association].room_keyframes, det_infinite_room.room_keyframes);

      // mergeMarkerArrays(rooms_vec[finite_room_data_association], cluster_array);
      // setRoomCenter(rooms_vec[finite_room_data_association], room_center);
      setLargerPlanes(rooms_vec[finite_room_data_association], "y", det_infinite_room);
      return duplicate_found;

    }
    else {
      room_data_association = associate_infinite_rooms(graph_slam,
                                                    plane_type,
                                                     room_center,
                                                     room_length,
                                                     cluster_array,
                                                     (found_plane1->second),
                                                     (found_plane2->second),
                                                     x_vert_planes,
                                                     y_vert_planes,
                                                     x_infinite_rooms,
                                                     y_infinite_rooms,
                                                     detected_mapped_plane_pairs);
    }

    if ((y_infinite_rooms.empty() || room_data_association == -1)) {
      std::cout << "found an Y infinite_room with pre pose "
                << room_center.translation() << " between plane "
                << room_plane1_pair.plane_unflipped.coeffs() << " and plane "
                << room_plane2_pair.plane_unflipped.coeffs() << std::endl;

      room_data_association = graph_slam->retrieve_local_nbr_of_vertices();
      room_node = graph_slam->add_room_node(room_center);
      // cluster_center_node = graph_slam->add_room_node(cluster_center);
      // cluster_center_node->setFixed(true);

      InfiniteRooms det_infinite_room;
      det_infinite_room.id = room_data_association;
      room_id = det_infinite_room.id;
      det_infinite_room.plane1 = room_plane1_pair.plane_unflipped;
      det_infinite_room.plane2 = room_plane2_pair.plane_unflipped;
      det_infinite_room.plane1_id = room_plane1_pair.plane_id;
      det_infinite_room.plane2_id = room_plane2_pair.plane_id;
      det_infinite_room.cluster_array = cluster_array;

      det_infinite_room.room_max = room_center.translation().x()+room_length.x/2;
      det_infinite_room.room_min = room_center.translation().x()-room_length.x/2;
      // add keyframe
      // det_infinite_room.room_keyframes = get_infinite_room_keyframes(det_infinite_room, y_vert_planes, keyframes);
      det_infinite_room.room_keyframes = get_infinite_room_keyframes(det_infinite_room, y_vert_planes, keyframes);
      
      det_infinite_room.old_roompart_keyframes.clear();
      det_infinite_room.new_roompart_keyframes.clear();
      det_infinite_room.new_roompart_keyframes.insert( {0,{0,0,det_infinite_room.room_keyframes}});

      // det_infinite_room.cluster_center_node = cluster_center_node;
      det_infinite_room.node = room_node;
      det_infinite_room.plane1_node = (found_plane1->second).plane_node;
      det_infinite_room.plane2_node = (found_plane2->second).plane_node;
      det_infinite_room.local_graph = std::make_shared<GraphSLAM>();
      y_infinite_rooms.insert({det_infinite_room.id, det_infinite_room});

      auto edge_room_plane =
          graph_slam->add_room_2planes_edge(room_node,
                                            (found_plane1->second).plane_node,
                                            (found_plane2->second).plane_node,
                                            room_node,
                                            information_infinite_room_planes);
      graph_slam->add_robust_kernel(edge_room_plane, "Huber", 1.0);
    } else {
      // else 
      if (y_infinite_rooms.find(room_data_association)!=y_infinite_rooms.end()){
        /* add the edge between detected planes and the infinite_room */
        room_node = y_infinite_rooms[room_data_association].node;
        room_id = room_data_association;
        std::cout << "Matched det infinite_room Y with pre pose "
                  << room_center.translation() << " to mapped infinite_room with id "
                  << room_data_association << " and pose "
                  << room_node->estimate().translation() << std::endl;
        matched = true;

        update_old_roompart_keyframes(y_infinite_rooms[room_data_association].room_keyframes,
                        "y", y_infinite_rooms[room_data_association].room_min,  y_infinite_rooms[room_data_association].room_max, roompart_length,
                        y_infinite_rooms[room_data_association].old_roompart_keyframes);

        InfiniteRooms det_infinite_room;
        det_infinite_room.id = room_data_association;
        det_infinite_room.plane1 = room_plane1_pair.plane_unflipped;
        det_infinite_room.plane2 = room_plane2_pair.plane_unflipped;
        det_infinite_room.plane1_id = room_plane1_pair.plane_id;
        det_infinite_room.plane2_id = room_plane2_pair.plane_id;

        det_infinite_room.room_max = room_center.translation().x()+room_length.x/2;
        det_infinite_room.room_min = room_center.translation().x()-room_length.x/2;
        // add keyframe
        // det_infinite_room.room_keyframes = get_infinite_room_keyframes(det_infinite_room, y_vert_planes, keyframes);
        // for (auto new_kf: y_infinite_rooms[room_data_association].new_roompart_keyframes){
        //   y_infinite_rooms[room_data_association].old_roompart_keyframes.insert(new_kf);
        // }
        // get_infinite_room_keyframes(det_infinite_room, y_vert_planes, keyframes,
        //             "y", det_infinite_room.room_min, det_infinite_room.room_max, roompart_length);
        det_infinite_room.room_keyframes = get_infinite_room_keyframes(det_infinite_room,y_vert_planes, keyframes);
        update_old_roompart_keyframes(det_infinite_room.room_keyframes,
                        "y", det_infinite_room.room_min, det_infinite_room.room_max, roompart_length,
                        det_infinite_room.new_roompart_keyframes);

        // for (auto &kf: y_infinite_rooms[room_data_association].room_keyframes) {
        //   std::get<2>(y_infinite_rooms[room_data_association].old_roompart_keyframes[0]).insert(kf); }
        // y_infinite_rooms[room_data_association].new_roompart_keyframes.clear();
        // y_infinite_rooms[room_data_association].new_roompart_keyframes.insert(
        //   {0,{0,0,det_infinite_room.room_keyframes}});
        mergeKeyframes(y_infinite_rooms[room_data_association].room_keyframes, det_infinite_room.room_keyframes);      
        
        y_infinite_rooms[room_data_association].room_max = max(y_infinite_rooms[room_data_association].room_max, det_infinite_room.room_max );
        y_infinite_rooms[room_data_association].room_min = min(y_infinite_rooms[room_data_association].room_min, det_infinite_room.room_min );

        // y_infinite_rooms[room_data_association].cluster_array = cluster_array;
        mergeMarkerArrays(y_infinite_rooms[room_data_association], "y", cluster_array);
        setRoomCenter(y_infinite_rooms[room_data_association], "y", room_center);
        setLargerPlanes(y_infinite_rooms[room_data_association], det_infinite_room);

        y_infinite_rooms[room_data_association].new_roompart_keyframes.clear();
        for (auto new_part: det_infinite_room.new_roompart_keyframes){
          y_infinite_rooms[room_data_association].new_roompart_keyframes.insert(new_part);
        }   

      }
      else if (x_infinite_rooms.find(room_data_association)!= x_infinite_rooms.end()) {
      //   return duplicate_found; 
        InfiniteRooms det_infinite_room;
        det_infinite_room.plane1 = room_plane1_pair.plane_unflipped;
        det_infinite_room.plane2 = room_plane2_pair.plane_unflipped;
        det_infinite_room.plane1_id = room_plane1_pair.plane_id;
        det_infinite_room.plane2_id = room_plane2_pair.plane_id;
        
        det_infinite_room.room_max = room_center.translation().x()+room_length.x/2;
        det_infinite_room.room_min = room_center.translation().x()-room_length.x/2;
        det_infinite_room.room_keyframes = get_infinite_room_keyframes(det_infinite_room, y_vert_planes, keyframes);
        
        
        room_id = room_data_association;
        std::cout << "Matched det infinite_room X with pre pose "
                  << room_center.translation() << " to mapped finite_room with id "
                  << room_data_association << " and pose "
                  << room_node->estimate().translation() << std::endl;
        matched = true;

        Rooms det_room;
        det_room.id = room_id;

        // mergeMarkerArrays(y_infinite_rooms[room_data_association].cluster_array, "y", cluster_array);
        Eigen::Isometry3d updated_room_center = Eigen::Isometry3d::Identity();
        updated_room_center.translation().x() = room_center.translation()(0);
        updated_room_center.translation().y() = 0.5*(x_infinite_rooms[room_data_association].room_max+x_infinite_rooms[room_data_association].room_min);
        updated_room_center.linear() = room_center.rotation();

        room_node = graph_slam->add_room_node(updated_room_center);
        
        det_room.cluster_array = cluster_array;

        det_room.plane_y1 = det_infinite_room.plane1;
        det_room.plane_y2 = det_infinite_room.plane2;
        det_room.plane_x1 = x_infinite_rooms[room_data_association].plane1;
        det_room.plane_x2 = x_infinite_rooms[room_data_association].plane2;
        det_room.plane_y1_id = det_infinite_room.plane1_id;
        det_room.plane_y2_id = det_infinite_room.plane2_id;
        det_room.plane_x1_id = x_infinite_rooms[room_data_association].plane1_id;
        det_room.plane_x2_id = x_infinite_rooms[room_data_association].plane2_id;
        det_room.plane_y1_node = (found_plane1->second).plane_node;
        det_room.plane_y2_node = (found_plane2->second).plane_node;
        det_room.plane_x1_node = x_infinite_rooms[room_data_association].plane1_node;
        det_room.plane_x2_node = x_infinite_rooms[room_data_association].plane2_node;

        det_room.room_x_max = det_infinite_room.room_max; 
        det_room.room_x_min = det_infinite_room.room_min;
        det_room.room_y_max = max(room_center.translation().y()+room_length.y/2, x_infinite_rooms[room_data_association].room_max);
        det_room.room_y_min = min(room_center.translation().y()-room_length.y/2, x_infinite_rooms[room_data_association].room_min);

        det_room.local_graph = std::make_shared<GraphSLAM>();
        det_room.node = room_node;
        rooms_vec.insert({det_room.id, det_room});
        
        for (auto &kf: x_infinite_rooms[room_data_association].room_keyframes) {
          std::get<2>(det_room.old_roompart_keyframes[0]).insert(kf); }
        
        mergeKeyframes(det_room.room_keyframes, x_infinite_rooms[room_data_association].room_keyframes);
        mergeKeyframes(det_room.room_keyframes, det_infinite_room.room_keyframes);

        det_room.new_roompart_keyframes.insert({0,{0,0,det_infinite_room.room_keyframes}});
        
        mergeMarkerArrays(det_room, x_infinite_rooms[room_data_association].cluster_array);
        mergeMarkerArrays(det_room, det_infinite_room.cluster_array);
    
        remove_mapped_infinite_room(PlaneUtils::plane_class::X_VERT_PLANE,
                                  graph_slam,
                                  x_infinite_rooms[room_data_association],
                                  x_infinite_rooms,
                                  y_infinite_rooms);
      }

      std::set<g2o::HyperGraph::Edge*> plane1_edges =
          (found_plane1->second).plane_node->edges();
      std::set<g2o::HyperGraph::Edge*> plane2_edges =
          (found_plane2->second).plane_node->edges();

      if (detected_mapped_plane_pairs[0].first.id !=
          detected_mapped_plane_pairs[0].second.id) {
        if (!MapperUtils::check_plane_ids(
                plane1_edges, detected_mapped_plane_pairs[0].second.plane_node)
          && PlaneUtils::plane_dot_product(detected_mapped_plane_pairs[0].first.plane_node
                , detected_mapped_plane_pairs[0].second.plane_node)> 0.9) {
          auto edge_planes = graph_slam->add_2planes_edge(
              detected_mapped_plane_pairs[0].first.plane_node,
              detected_mapped_plane_pairs[0].second.plane_node,
              information_2planes);
          
          // Eigen::Vector4d mea(0,0,0,0);
          // auto edge_planes = graph_slam->add_plane_identity_edge(
          //     detected_mapped_plane_pairs[0].first.plane_node,
          //     detected_mapped_plane_pairs[0].second.plane_node,
          //     mea,
          //     information_planes_identity);
          graph_slam->add_robust_kernel(edge_planes, "Huber", 1.0);
          duplicate_found = true;
          std::cout << "Adding new y1 plane " << std::endl;
        }
      }
      if (detected_mapped_plane_pairs[1].first.id !=
          detected_mapped_plane_pairs[1].second.id) {
        if (!MapperUtils::check_plane_ids(
                plane2_edges, detected_mapped_plane_pairs[1].second.plane_node)
          && PlaneUtils::plane_dot_product(detected_mapped_plane_pairs[1].first.plane_node
                , detected_mapped_plane_pairs[1].second.plane_node)> 0.9) {
          auto edge_planes = graph_slam->add_2planes_edge(
              detected_mapped_plane_pairs[1].first.plane_node,
              detected_mapped_plane_pairs[1].second.plane_node,
              information_2planes);
          
          // Eigen::Vector4d mea(0,0,0,0);
          // auto edge_planes = graph_slam->add_plane_identity_edge(
          //     detected_mapped_plane_pairs[1].first.plane_node,
          //     detected_mapped_plane_pairs[1].second.plane_node,
          //     mea,
          //     information_planes_identity);
          // std::cout<<"add_plane_identity_edge"<<std::endl;
          graph_slam->add_robust_kernel(edge_planes, "Huber", 1.0);
          // std::cout<<"add_robust_kernel"<<std::endl;
          duplicate_found = true;
          std::cout << "Adding new y2 plane " << std::endl;
        }
      }
    }
  }

  return duplicate_found;
}

int InfiniteRoomMapper::associate_rooms(
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
        detected_mapped_plane_pairs) {
  bool plane1_min_segment = false, plane2_min_segment = false;
  Eigen::Quaterniond room_quat;
  bool duplicate_found = false;
  int data_association = -1;

  if (plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
    // check the distance with the current room vector
    Rooms matched_room;
    float min_dist_x_inf_room_room = 100;
    float matched_y_overlap = 0;
    for (const auto& current_room : rooms_vec) {
      if ((plane1.id == current_room.second.plane_x1_id ||
           plane1.id == current_room.second.plane_x2_id) &&
          (plane2.id == current_room.second.plane_x1_id ||
           plane2.id == current_room.second.plane_x2_id)) {
        min_dist_x_inf_room_room = 0;
        matched_room = current_room.second;
        break;
      }
      float dist_x_inf_room_room = abs(room_center.translation()(0) -
                       current_room.second.node->estimate().translation()(0));
      // geometry_msgs::msg::Point length = InfiniteRoomMapper::plane_length(current_room.second.cluster_array);
      // std::cout<<"####room_data.room_length.y "<<room_data.room_length.y<<" current_room_width "<<length.y<<std::endl;
      double finite_room_x_max = current_room.second.room_x_max;
      double finite_room_x_min = current_room.second.room_x_min;
      double finite_room_y_max = current_room.second.room_y_max;
      double finite_room_y_min = current_room.second.room_y_min;

      float x_overlap = max(0.0,
                          min(room_center.translation()(0) + room_length.x/2.0, finite_room_x_max) -
                            max(room_center.translation()(0)- room_length.x/2.0, finite_room_x_min));
      float y_overlap = max(0.0,
                          min(room_center.translation()(1) + room_length.y/2.0, finite_room_y_max) -
                            max(room_center.translation()(1)- room_length.y/2.0, finite_room_y_min));
      // if (x_overlap>0 && y_overlap>0 && dist_x_inf_room_room < min_dist_x_inf_room_room) {
      if (x_overlap>1.0 && y_overlap>1.0 && abs(x_overlap-room_length.x)<1.0 && abs(y_overlap-room_length.y)<1.0&& dist_x_inf_room_room < min_dist_x_inf_room_room) {
        if (abs(room_center.translation()(0) + room_length.x/2.0 - finite_room_x_max)<5.0 && abs(room_center.translation()(0)- room_length.x/2.0 - finite_room_x_min)<5.0) {
          min_dist_x_inf_room_room = dist_x_inf_room_room;
          matched_room = current_room.second;
          matched_y_overlap = y_overlap;
        }
      }
    }

    // if ( matched_y_overlap < matched_finite_room_y) {
    if (min_dist_x_inf_room_room < room_dist_threshold) {
      // room_center.translation()(0) = 0.5*(room_center.translation()(0)+matched_room.node->estimate().translation()(0));
      // room_center.translation()(1) = 0.5*(room_center.translation()(1)+matched_room.node->estimate().translation()(1))
      // room_length = room_length.y +length.y -matched_y_overlap;

      std::vector<std::pair<VerticalPlanes, VerticalPlanes>>
          current_detected_mapped_plane_pairs;
      std::pair<VerticalPlanes, VerticalPlanes> x1_detected_mapped_plane_pair;
      std::pair<VerticalPlanes, VerticalPlanes> x2_detected_mapped_plane_pair;

      auto found_mapped_plane1 = x_vert_planes.find(matched_room.plane_x1_id);
      auto found_mapped_plane2 = x_vert_planes.find(matched_room.plane_x2_id);

      if (plane1.id == (found_mapped_plane1->second).id ||
          plane1.id == (found_mapped_plane2->second).id) {
        plane1_min_segment = true;
        x1_detected_mapped_plane_pair.first = plane1;
        x1_detected_mapped_plane_pair.second = plane1;
      } else if ((plane1).plane_node->estimate().coeffs().head(3).dot(
                     (found_mapped_plane1->second)
                         .plane_node->estimate()
                         .coeffs()
                         .head(3)) > 0) {
        plane1_min_segment = PlaneUtils::check_point_neighbours(
            (found_mapped_plane1->second).cloud_seg_map, plane1.cloud_seg_map);
        if (plane1_min_segment) (found_mapped_plane1->second).plane_node->setId(plane1.id); ///
        x1_detected_mapped_plane_pair.first = plane1;
        x1_detected_mapped_plane_pair.second = (found_mapped_plane1->second);
      } else {
        plane1_min_segment = PlaneUtils::check_point_neighbours(
            (found_mapped_plane2->second).cloud_seg_map, plane1.cloud_seg_map);
        if (plane1_min_segment) (found_mapped_plane2->second).plane_node->setId(plane1.id); ///
        x1_detected_mapped_plane_pair.first = plane1;
        x1_detected_mapped_plane_pair.second = (found_mapped_plane2->second);
      }
      current_detected_mapped_plane_pairs.push_back(x1_detected_mapped_plane_pair);

      if (plane2.id == (found_mapped_plane1->second).id ||
          plane2.id == (found_mapped_plane2->second).id) {
        plane2_min_segment = true;
        x2_detected_mapped_plane_pair.first = plane2;
        x2_detected_mapped_plane_pair.second = plane2;
      } else if ((plane2).plane_node->estimate().coeffs().head(3).dot(
                     (found_mapped_plane1->second)
                         .plane_node->estimate()
                         .coeffs()
                         .head(3)) > 0) {
        plane2_min_segment = PlaneUtils::check_point_neighbours(
            (found_mapped_plane1->second).cloud_seg_map, plane2.cloud_seg_map);
        if (plane2_min_segment) (found_mapped_plane1->second).plane_node->setId(plane2.id); ///
        x2_detected_mapped_plane_pair.first = plane2;
        x2_detected_mapped_plane_pair.second = (found_mapped_plane1->second);
      } else {
        plane2_min_segment = PlaneUtils::check_point_neighbours(
            (found_mapped_plane2->second).cloud_seg_map, plane2.cloud_seg_map);
        if (plane2_min_segment) (found_mapped_plane2->second).plane_node->setId(plane2.id); ///
        x2_detected_mapped_plane_pair.first = plane2;
        x2_detected_mapped_plane_pair.second = (found_mapped_plane2->second);
      }
      current_detected_mapped_plane_pairs.push_back(x2_detected_mapped_plane_pair);

      if (plane1_min_segment || plane2_min_segment) {
        std::cout << "Matched x infinite_room with finite room" << std::endl;
        // std::cout<<"min_dist_x_inf_room_room "<<min_dist_x_inf_room_room<<" matched_y_overlap "<<matched_y_overlap<<std::endl;
        data_association = matched_room.id;
        // matched = true;
        detected_mapped_plane_pairs = current_detected_mapped_plane_pairs;
      }
    }
     return data_association;
  }

  else if (plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
    float min_dist_y_inf_room_room = 100;
    float matched_x_overlap = 0;
    Rooms matched_room;
    for (const auto& current_room : rooms_vec) {
      if ((plane1.id == current_room.second.plane_y1_id ||
           plane1.id == current_room.second.plane_y2_id) &&
          (plane2.id == current_room.second.plane_y1_id ||
           plane2.id == current_room.second.plane_y2_id)) {
        min_dist_y_inf_room_room = 0;
        matched_room = current_room.second;
        break;
      }
      float dist_y_inf_room_room = abs(room_center.translation()(1) -
                       current_room.second.node->estimate().translation()(1));
      
      double finite_room_x_max = current_room.second.room_x_max;
      double finite_room_x_min = current_room.second.room_x_min;
      double finite_room_y_max = current_room.second.room_y_max;
      double finite_room_y_min = current_room.second.room_y_min;

      float x_overlap = max(0.0,
                          min(room_center.translation()(0) + room_length.x/2.0, finite_room_x_max) -
                            max(room_center.translation()(0)- room_length.x/2.0, finite_room_x_min));
      float y_overlap = max(0.0,
                          min(room_center.translation()(1) + room_length.y/2.0, finite_room_y_max) -
                            max(room_center.translation()(1)- room_length.y/2.0, finite_room_y_min));
      
      // if (dist_y_inf_room_room < min_dist_y_inf_room_room && x_overlap>0 && y_overlap>0) {
      if (x_overlap>0 && y_overlap>0 && abs(x_overlap-room_length.x)<1.0 && abs(y_overlap-room_length.y)<1.0&& dist_y_inf_room_room < min_dist_y_inf_room_room ) {
        min_dist_y_inf_room_room = dist_y_inf_room_room;
        matched_room = current_room.second;
        matched_x_overlap = x_overlap;
      }
    }
    
    if (min_dist_y_inf_room_room < room_dist_threshold) {
      std::vector<std::pair<VerticalPlanes, VerticalPlanes>> current_detected_mapped_plane_pairs;
      std::pair<VerticalPlanes, VerticalPlanes> y1_detected_mapped_plane_pair;
      std::pair<VerticalPlanes, VerticalPlanes> y2_detected_mapped_plane_pair;
      auto found_mapped_plane1 = y_vert_planes.find(matched_room.plane_y1_id);
      auto found_mapped_plane2 = y_vert_planes.find(matched_room.plane_y2_id);

      if (plane1.id == (found_mapped_plane1->second).id ||
          plane1.id == (found_mapped_plane2->second).id) {
        plane1_min_segment = true;
        y1_detected_mapped_plane_pair.first = plane1;
        y1_detected_mapped_plane_pair.second = plane1;
      } else if ((plane1).plane_node->estimate().coeffs().head(3).dot(
                     (found_mapped_plane1->second)
                         .plane_node->estimate()
                         .coeffs()
                         .head(3)) > 0) {
        plane1_min_segment = PlaneUtils::check_point_neighbours(
            (found_mapped_plane1->second).cloud_seg_map, plane1.cloud_seg_map);
        // if (plane1_min_segment) (found_mapped_plane1->second).plane_node->setId(plane1.id); ///
        y1_detected_mapped_plane_pair.first = plane1;
        y1_detected_mapped_plane_pair.second = (found_mapped_plane1->second);
      } else {
        plane1_min_segment = PlaneUtils::check_point_neighbours(
            (found_mapped_plane2->second).cloud_seg_map, plane1.cloud_seg_map);
        // if (plane1_min_segment) (found_mapped_plane2->second).plane_node->setId(plane1.id); ///
        y1_detected_mapped_plane_pair.first = plane1;
        y1_detected_mapped_plane_pair.second = (found_mapped_plane2->second);
      }
      current_detected_mapped_plane_pairs.push_back(y1_detected_mapped_plane_pair);

      if (plane2.id == (found_mapped_plane1->second).id ||
          plane2.id == (found_mapped_plane2->second).id) {
        plane2_min_segment = true;
        y2_detected_mapped_plane_pair.first = plane2;
        y2_detected_mapped_plane_pair.second = plane2;
      } else if ((plane2).plane_node->estimate().coeffs().head(3).dot(
                     (found_mapped_plane1->second)
                         .plane_node->estimate()
                         .coeffs()
                         .head(3)) > 0) {
        plane2_min_segment = PlaneUtils::check_point_neighbours(
            (found_mapped_plane1->second).cloud_seg_map, plane2.cloud_seg_map);
        // if (plane2_min_segment) (found_mapped_plane1->second).plane_node->setId(plane2.id); ///
        y2_detected_mapped_plane_pair.first = plane2;
        y2_detected_mapped_plane_pair.second = (found_mapped_plane1->second);
      } else {
        plane2_min_segment = PlaneUtils::check_point_neighbours(
            (found_mapped_plane2->second).cloud_seg_map, plane2.cloud_seg_map);
        // if (plane2_min_segment) (found_mapped_plane2->second).plane_node->setId(plane2.id); ///
        y2_detected_mapped_plane_pair.first = plane2;
        y2_detected_mapped_plane_pair.second = (found_mapped_plane2->second);
      }
      current_detected_mapped_plane_pairs.push_back(y2_detected_mapped_plane_pair);

      if (plane1_min_segment || plane2_min_segment) {
        std::cout << "Matched y infinite_room with finite room"<< std::endl;
        // std::cout<<"min_dist_y_inf_room_room "<<min_dist_y_inf_room_room<<" matched_x_overlap "<<matched_x_overlap<<std::endl;
        data_association = matched_room.id;
        // matched = true;
        detected_mapped_plane_pairs = current_detected_mapped_plane_pairs;
      }
    }
      return data_association;
    }

  }
int InfiniteRoomMapper::associate_infinite_rooms(
    std::shared_ptr<GraphSLAM>& graph_slam,
    const int& plane_type,
    const Eigen::Isometry3d& room_center,
    const geometry_msgs::msg::Point &room_length,
    const visualization_msgs::msg::MarkerArray& cluster_array,
    const VerticalPlanes& plane1,
    const VerticalPlanes& plane2,
    const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    std::unordered_map<int, InfiniteRooms>& x_infinite_rooms,
    std::unordered_map<int, InfiniteRooms>& y_infinite_rooms,
    std::vector<std::pair<VerticalPlanes, VerticalPlanes>>&
        detected_mapped_plane_pairs) {
  float min_dist = 100;
  int min_id = 1000000;
  bool plane1_min_segment = false, plane2_min_segment = false;

  int data_association;
  data_association = -1;
  vector<InfiniteRooms> matched_cand;
  if (plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
    for (const auto& x_inf_room : x_infinite_rooms) {
      float dist = sqrt(pow(room_center.translation()(0) -
                                x_inf_room.second.node->estimate().translation()(0),
                            2));
      
      geometry_msgs::msg::Point length = InfiniteRoomMapper::plane_length(x_inf_room.second.cluster_array);
      float x_overlap = max(0.0,
                            min(room_center.translation()(0) + room_length.x/2.0, x_inf_room.second.node->estimate().translation()(0) + length.x/2.0) -
                             max(room_center.translation()(0) - room_length.x/2.0, x_inf_room.second.node->estimate().translation()(0) - length.x/2.0));
      
      float y_overlap = max(0.0,
                            min(room_center.translation()(1) + room_length.y/2.0,  x_inf_room.second.room_max) - 
                             max(room_center.translation()(1) - room_length.y/2.0,  x_inf_room.second.room_min));
      // std::cout<<"associate_infinite_rooms: "<<x_inf_room.second.id<<" dist "<<dist<<" overlap "<<x_overlap<<" "<<y_overlap<<", plane1_min_segment || plane2_min_segment "<< plane1_min_segment <<" "<<plane2_min_segment<<std::endl;
      
      ///
      geometry_msgs::msg::Point new_room_center;
      new_room_center.x = 0.5* (room_center.translation()(0) + x_inf_room.second.node->estimate().translation()(0));
      new_room_center.y = 0.5* (room_center.translation()(1) + x_inf_room.second.node->estimate().translation()(1));
      
      bool centroid_inside = false;
      centroid_inside = MapperUtils::extract_centroid_location(cluster_array, new_room_center);
      if (!centroid_inside)
        centroid_inside = MapperUtils::extract_centroid_location(x_inf_room.second.cluster_array, new_room_center);
      if (!centroid_inside) {
        // std::cout<<"== room center avg outside clusters"<<std::endl;
        continue;
      }

      std::vector<std::pair<VerticalPlanes, VerticalPlanes>>
          current_detected_mapped_plane_pairs;
      std::pair<VerticalPlanes, VerticalPlanes> x1_detected_mapped_plane_pair;
      std::pair<VerticalPlanes, VerticalPlanes> x2_detected_mapped_plane_pair;

      auto found_mapped_plane1 = x_vert_planes.find(x_inf_room.second.plane1_id);
      auto found_mapped_plane2 = x_vert_planes.find(x_inf_room.second.plane2_id);
      
      if (plane1.id == (found_mapped_plane1->second).id ||
          plane1.id == (found_mapped_plane2->second).id) {
        plane1_min_segment = true;
        x1_detected_mapped_plane_pair.first = plane1;
        x1_detected_mapped_plane_pair.second = plane1;
      } else if ((plane1).plane_node->estimate().coeffs().head(3).dot(
                     (found_mapped_plane1->second)
                         .plane_node->estimate()
                         .coeffs()
                         .head(3)) > 0) {
        plane1_min_segment = PlaneUtils::check_point_neighbours(
            (found_mapped_plane1->second).cloud_seg_map, plane1.cloud_seg_map);
        x1_detected_mapped_plane_pair.first = plane1;
        x1_detected_mapped_plane_pair.second = (found_mapped_plane1->second);
      } else {
        plane1_min_segment = PlaneUtils::check_point_neighbours(
            (found_mapped_plane2->second).cloud_seg_map, plane1.cloud_seg_map);
        x1_detected_mapped_plane_pair.first = plane1;
        x1_detected_mapped_plane_pair.second = (found_mapped_plane2->second);
      }
      current_detected_mapped_plane_pairs.push_back(x1_detected_mapped_plane_pair);

      if (plane2.id == (found_mapped_plane1->second).id ||
          plane2.id == (found_mapped_plane2->second).id) {
        plane2_min_segment = true;
        x2_detected_mapped_plane_pair.first = plane2;
        x2_detected_mapped_plane_pair.second = plane2;
      } else if ((plane2).plane_node->estimate().coeffs().head(3).dot(
                     (found_mapped_plane1->second)
                         .plane_node->estimate()
                         .coeffs()
                         .head(3)) > 0) {
        plane2_min_segment = PlaneUtils::check_point_neighbours(
            (found_mapped_plane1->second).cloud_seg_map, plane2.cloud_seg_map);
        x2_detected_mapped_plane_pair.first = plane2;
        x2_detected_mapped_plane_pair.second = (found_mapped_plane1->second);
      } else {
        plane2_min_segment = PlaneUtils::check_point_neighbours(
            (found_mapped_plane2->second).cloud_seg_map, plane2.cloud_seg_map);
        x2_detected_mapped_plane_pair.first = plane2;
        x2_detected_mapped_plane_pair.second = (found_mapped_plane2->second);
      }
      current_detected_mapped_plane_pairs.push_back(x2_detected_mapped_plane_pair);

      std::cout<<"x_inf_room "<<dist<<" "<<x_overlap<<" "<<y_overlap<<" "<<plane1_min_segment<<" "<<plane2_min_segment<<std::endl;
      // if (dist < min_dist && (plane1_min_segment || plane2_min_segment)) {
      // if (dist<min_dist && (plane1_min_segment || plane2_min_segment || y_overlap>0.0)) {
      if (dist<infinite_room_dist_threshold && x_inf_room.second.id<min_id 
          && (abs(room_center.translation()(0) + room_length.x/2.0 - (x_inf_room.second.node->estimate().translation()(0) + length.x/2.0))<5.0 
              && abs(room_center.translation()(0)- room_length.x/2.0 - (x_inf_room.second.node->estimate().translation()(0) - length.x/2.0))<5.0) 
          && ((x_overlap>0 && y_overlap>0 && (plane1_min_segment || plane2_min_segment)) 
              || (plane1_min_segment && plane2_min_segment)) ) {
        min_dist = dist;
        min_id = x_inf_room.second.id;
        data_association = x_inf_room.second.id;

        if ((plane1.id == (found_mapped_plane1->second).id || plane1.id == (found_mapped_plane2->second).id)
          && (plane2.id == (found_mapped_plane1->second).id || plane2.id == (found_mapped_plane2->second).id))
          matched_cand.push_back(x_inf_room.second);
        detected_mapped_plane_pairs = current_detected_mapped_plane_pairs;
        RCLCPP_DEBUG(
            node_obj->get_logger(), "infinite_room planes", "dist x room %f", dist);
      }
    }
    for (auto &matched_x_infinite_room: matched_cand){
      if (matched_x_infinite_room.id!=min_id) {
        std::cout<<"matched_x_infinite_room.id "<<matched_x_infinite_room.id<<std::endl;
        if (x_infinite_rooms.find(matched_x_infinite_room.id) == x_infinite_rooms.end()
         || x_infinite_rooms.find(min_id) == x_infinite_rooms.end()) continue;
        mergeKeyframes(x_infinite_rooms[min_id].room_keyframes, matched_x_infinite_room.room_keyframes);      
        mergeMarkerArrays(x_infinite_rooms[min_id], "x", matched_x_infinite_room.cluster_array);
        setRoomCenter(x_infinite_rooms[min_id], "x", room_center);
        remove_mapped_infinite_room(PlaneUtils::plane_class::X_VERT_PLANE,
                                  graph_slam,
                                  matched_x_infinite_room,
                                  x_infinite_rooms,
                                  y_infinite_rooms);
      }
    }
    for (const auto& y_inf_room : y_infinite_rooms) {
      geometry_msgs::msg::Point length = InfiniteRoomMapper::plane_length(y_inf_room.second.cluster_array);
      float dist = sqrt(pow(room_center.translation()(0) -
                              y_inf_room.second.node->estimate().translation()(0),
                          2)+pow(room_center.translation()(1) -
                              y_inf_room.second.node->estimate().translation()(1),
                          2));
      float x_overlap = max(0.0,
                            min(room_center.translation()(0) + room_length.x/2.0, y_inf_room.second.room_max) -
                             max(room_center.translation()(0) - room_length.x/2.0, y_inf_room.second.room_min));
      float y_overlap = max(0.0,
                            min(room_center.translation()(1) + room_length.y/2.0,  y_inf_room.second.node->estimate().translation()(1) + length.y/2.0) - 
                             max(room_center.translation()(1) - room_length.y/2.0,  y_inf_room.second.node->estimate().translation()(1) - length.y/2.0));
      std::cout<<"associate y_infinite_rooms: "<<y_inf_room.second.id<<" dist "<<dist<<" overlap "<<x_overlap<<" "<<y_overlap<<", length "<<length.x<<" "<<length.y<<std::endl;
      if (dist<1.0 && x_overlap>1.0 && y_overlap>1.0) {
        data_association = y_inf_room.second.id;   
        return data_association;
      }
    }
    // //merge x_inf and y_inf into finite_room (maybe in visuzalization.cpp?)
  }

  if (plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
    for (const auto& y_inf_room : y_infinite_rooms) {
      float dist = sqrt(pow(room_center.translation()(1) -
                                y_inf_room.second.node->estimate().translation()(1),
                            2));
      geometry_msgs::msg::Point length = InfiniteRoomMapper::plane_length(y_inf_room.second.cluster_array);

      float x_overlap = max(0.0,
                            min(room_center.translation()(0) + room_length.x/2.0, y_inf_room.second.room_max) -
                             max(room_center.translation()(0) - room_length.x/2.0, y_inf_room.second.room_min));
      float y_overlap = max(0.0,
                            min(room_center.translation()(1) + room_length.y/2.0,  y_inf_room.second.node->estimate().translation()(1) + length.y/2.0) - 
                             max(room_center.translation()(1) - room_length.y/2.0,  y_inf_room.second.node->estimate().translation()(1) - length.y/2.0));
      // std::cout<<"associate_infinite_rooms: "<<y_inf_room.second.id<<" dist "<<dist<<" overlap "<<x_overlap<<" "<<y_overlap<<", plane1_min_segment || plane2_min_segment "<< plane1_min_segment <<" "<<plane2_min_segment<<std::endl;
      
      ///
      geometry_msgs::msg::Point new_room_center;
      new_room_center.x = 0.5* (room_center.translation()(0) + y_inf_room.second.node->estimate().translation()(0));
      new_room_center.y = 0.5* (room_center.translation()(1) + y_inf_room.second.node->estimate().translation()(1));
      
      bool centroid_inside = false;
      centroid_inside = MapperUtils::extract_centroid_location(cluster_array, new_room_center);
      if (!centroid_inside)
        centroid_inside = MapperUtils::extract_centroid_location(y_inf_room.second.cluster_array, new_room_center);
      if (!centroid_inside) {
        // std::cout<<"== room center avg outside clusters"<<std::endl;
        continue;
      }

      std::vector<std::pair<VerticalPlanes, VerticalPlanes>>
          current_detected_mapped_plane_pairs;
      std::pair<VerticalPlanes, VerticalPlanes> y1_detected_mapped_plane_pair;
      std::pair<VerticalPlanes, VerticalPlanes> y2_detected_mapped_plane_pair;
      auto found_mapped_plane1 = y_vert_planes.find(y_inf_room.second.plane1_id);
      auto found_mapped_plane2 = y_vert_planes.find(y_inf_room.second.plane2_id);

      if (plane1.id == (found_mapped_plane1->second).id ||
          plane1.id == (found_mapped_plane2->second).id) {
        plane1_min_segment = true;
        y1_detected_mapped_plane_pair.first = plane1;
        y1_detected_mapped_plane_pair.second = plane1;
      } else if ((plane1).plane_node->estimate().coeffs().head(3).dot(
                     (found_mapped_plane1->second)
                         .plane_node->estimate()
                         .coeffs()
                         .head(3)) > 0) {
        plane1_min_segment = PlaneUtils::check_point_neighbours(
            (found_mapped_plane1->second).cloud_seg_map, plane1.cloud_seg_map);
        // if (plane1_min_segment) plane1.plane_node->setId((found_mapped_plane1->second).id); //
        y1_detected_mapped_plane_pair.first = plane1;
        y1_detected_mapped_plane_pair.second = (found_mapped_plane1->second);
      } else {
        plane1_min_segment = PlaneUtils::check_point_neighbours(
            (found_mapped_plane2->second).cloud_seg_map, plane1.cloud_seg_map);
        // if (plane1_min_segment) plane1.plane_node->setId((found_mapped_plane2->second).id);
        y1_detected_mapped_plane_pair.first = plane1;
        y1_detected_mapped_plane_pair.second = (found_mapped_plane2->second);
      }
      current_detected_mapped_plane_pairs.push_back(y1_detected_mapped_plane_pair);

      if (plane2.id == (found_mapped_plane1->second).id ||
          plane2.id == (found_mapped_plane2->second).id) {
        plane2_min_segment = true;
        y2_detected_mapped_plane_pair.first = plane2;
        y2_detected_mapped_plane_pair.second = plane2;
      } else if ((plane2).plane_node->estimate().coeffs().head(3).dot(
                     (found_mapped_plane1->second)
                         .plane_node->estimate()
                         .coeffs()
                         .head(3)) > 0) {
        plane2_min_segment = PlaneUtils::check_point_neighbours(
            (found_mapped_plane1->second).cloud_seg_map, plane2.cloud_seg_map);
        // if (plane2_min_segment) plane2.plane_node->setId((found_mapped_plane1->second).id);
        y2_detected_mapped_plane_pair.first = plane2;
        y2_detected_mapped_plane_pair.second = (found_mapped_plane1->second);
      } else {
        plane2_min_segment = PlaneUtils::check_point_neighbours(
            (found_mapped_plane2->second).cloud_seg_map, plane2.cloud_seg_map);
        // if (plane2_min_segment) plane2.plane_node->setId((found_mapped_plane2->second).id);
        y2_detected_mapped_plane_pair.first = plane2;
        y2_detected_mapped_plane_pair.second = (found_mapped_plane2->second);
      }
      current_detected_mapped_plane_pairs.push_back(y2_detected_mapped_plane_pair);
      // std::cout<<"y_inf_room "<<x_overlap<<" "<<y_overlap<<" "<<plane1_min_segment<<" "<<plane2_min_segment<<std::endl;
      // if (dist < min_dist && (plane1_min_segment || plane2_min_segment)) {
      if (dist<infinite_room_dist_threshold && y_inf_room.second.id<min_id 
          && (abs(room_center.translation()(1) + room_length.y/2.0 - (y_inf_room.second.node->estimate().translation()(1) + length.y/2.0))<5.0 
              && abs(room_center.translation()(1)- room_length.y/2.0 - (y_inf_room.second.node->estimate().translation()(1) - length.y/2.0))<5.0) 
          && ((x_overlap>1.0 && y_overlap>1.0 && (plane1_min_segment || plane2_min_segment))
              || (plane1_min_segment && plane2_min_segment)) ) {
        min_dist = dist;
        min_id = y_inf_room.second.id;
        data_association = y_inf_room.second.id;
        if ((plane1.id == (found_mapped_plane1->second).id || plane1.id == (found_mapped_plane2->second).id)
           && (plane2.id == (found_mapped_plane1->second).id || plane2.id == (found_mapped_plane2->second).id))
           matched_cand.push_back(y_inf_room.second);
        detected_mapped_plane_pairs = current_detected_mapped_plane_pairs;
        RCLCPP_DEBUG(
            node_obj->get_logger(), "infinite_room planes", "dist y room %f", dist);
      }
    }
    for (auto &matched_y_infinite_room: matched_cand){
      if (matched_y_infinite_room.id!=min_id) {
        if (y_infinite_rooms.find(matched_y_infinite_room.id) == y_infinite_rooms.end()
         || y_infinite_rooms.find(min_id) == y_infinite_rooms.end()) continue;
        mergeKeyframes(y_infinite_rooms[min_id].room_keyframes, matched_y_infinite_room.room_keyframes);      
        mergeMarkerArrays(y_infinite_rooms[min_id], "y", matched_y_infinite_room.cluster_array);
        remove_mapped_infinite_room(PlaneUtils::plane_class::Y_VERT_PLANE,
                                  graph_slam,
                                  matched_y_infinite_room,
                                  x_infinite_rooms,
                                  y_infinite_rooms);
      }
    }
    for (const auto& x_inf_room : x_infinite_rooms) {
      geometry_msgs::msg::Point length = InfiniteRoomMapper::plane_length(x_inf_room.second.cluster_array);
      float dist = sqrt(pow(room_center.translation()(0) -
                              x_inf_room.second.node->estimate().translation()(0),
                          2)+pow(room_center.translation()(1) -
                              x_inf_room.second.node->estimate().translation()(1),
                          2));
      float x_overlap = max(0.0,
                            min(room_center.translation()(0) + room_length.x/2.0, x_inf_room.second.node->estimate().translation()(0) + length.x/2.0) -
                             max(room_center.translation()(0) - room_length.x/2.0, x_inf_room.second.node->estimate().translation()(0) - length.x/2.0));
      float y_overlap = max(0.0,
                            min(room_center.translation()(1) + room_length.y/2.0, x_inf_room.second.room_max) - 
                             max(room_center.translation()(1) - room_length.y/2.0, x_inf_room.second.room_min) );
      // std::cout<<"associate_infinite_rooms: "<<y_inf_room.second.id<<" dist "<<dist<<" overlap "<<x_overlap<<" "<<y_overlap<<", plane1_min_segment || plane2_min_segment "<< plane1_min_segment <<" "<<plane2_min_segment<<std::endl;
      if (dist<1 && x_overlap>1 && y_overlap>1) {
        data_association = x_inf_room.second.id;   
      }
    }
  }

  // RCLCPP_DEBUG(node_obj->get_logger(),"infinite_room planes", "min dist %f",
  // min_dist);
  if (min_dist > infinite_room_dist_threshold) data_association = -1;
  if (data_association>30000) data_association = -1;
  // std::cout<<"data_association "<<data_association<<std::endl;
  return data_association;
}

bool InfiniteRoomMapper::check_infinite_room_ids(
    const int plane_type,
    const std::set<g2o::HyperGraph::Edge*>& plane_edges,
    const g2o::VertexRoom* room_node) {
  for (auto edge_itr = plane_edges.begin(); edge_itr != plane_edges.end(); ++edge_itr) {
    if (plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
      g2o::EdgeRoom2Planes* edge_infinite_room_planes =
          dynamic_cast<g2o::EdgeRoom2Planes*>(*edge_itr);
      if (edge_infinite_room_planes) {
        g2o::VertexRoom* found_infinite_room_node =
            dynamic_cast<g2o::VertexRoom*>(edge_infinite_room_planes->vertices()[0]);
        if (found_infinite_room_node->id() == room_node->id()) return true;
      }
    }

    if (plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
      g2o::EdgeRoom2Planes* edge_infinite_room_planes =
          dynamic_cast<g2o::EdgeRoom2Planes*>(*edge_itr);
      if (edge_infinite_room_planes) {
        g2o::VertexRoom* found_infinite_room_node =
            dynamic_cast<g2o::VertexRoom*>(edge_infinite_room_planes->vertices()[0]);
        if (found_infinite_room_node->id() == room_node->id()) return true;
      }
    }
  }
  return false;
}


void InfiniteRoomMapper::remove_mapped_infinite_room(
    const int plane_type,
    std::shared_ptr<GraphSLAM>& graph_slam,
    s_graphs::InfiniteRooms matched_infinite_room,
    std::unordered_map<int, InfiniteRooms>& x_infinite_rooms,
    std::unordered_map<int, InfiniteRooms>& y_infinite_rooms) {
  std::set<g2o::HyperGraph::Edge*> edges = matched_infinite_room.node->edges();
  for (auto edge_itr = edges.begin(); edge_itr != edges.end(); ++edge_itr) {
    g2o::EdgeRoom2Planes* edge_room_2planes =
        dynamic_cast<g2o::EdgeRoom2Planes*>(*edge_itr);
    if (edge_room_2planes) {
      if (graph_slam->remove_room_2planes_edge(edge_room_2planes))
        // std::cout << "removed edge - room-2planes " << std::endl;
      continue;
    }
  }

  if (plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
    if (graph_slam->remove_room_node(matched_infinite_room.node)) {
      auto mapped_infinite_room = x_infinite_rooms.find(matched_infinite_room.id);
      x_infinite_rooms.erase(mapped_infinite_room);
      // std::cout << "removed overlapped x-infinite_room " << std::endl;
    }
  } else if (plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
    if (graph_slam->remove_room_node(matched_infinite_room.node)) {
      auto mapped_infinite_room = y_infinite_rooms.find(matched_infinite_room.id);
      y_infinite_rooms.erase(mapped_infinite_room);
      // std::cout << "removed overlapped y-infinite_room " << std::endl;
    }
  }
}
}  // namespace s_graphs
// traversable