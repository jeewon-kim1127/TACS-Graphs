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



#ifndef LOOP_DETECTOR_HPP
#define LOOP_DETECTOR_HPP

#include <g2o/types/slam3d/vertex_se3.h>

#include <boost/format.hpp>
#include <s_graphs/backend/graph_slam.hpp>
#include <s_graphs/common/keyframe.hpp>
#include <s_graphs/common/registrations.hpp>

namespace s_graphs {

/**
 * @brief Struct Loop
 */
struct Loop {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<Loop>;

  /**
   * @brief Constructor of struct Loop
   *
   * @param key1
   * @param key2
   * @param relpose
   */
  Loop(const KeyFrame::Ptr& key1,
       const KeyFrame::Ptr& key2,
       const Eigen::Matrix4f& relpose)
      : key1(key1), key2(key2), relative_pose(relpose) {}

 public:
  KeyFrame::Ptr key1;
  KeyFrame::Ptr key2;
  Eigen::Matrix4f relative_pose;
};

/**
 * @brief This class finds loops by scam matching and adds them to the pose graph
 */
class LoopDetector {
 public:
  typedef pcl::PointXYZI PointT;

  /**
   * @brief Constructor of the class LoopDetector
   *
   * @param node
   */
  LoopDetector(const rclcpp::Node::SharedPtr node) {
    distance_thresh =
        node->get_parameter("distance_thresh").get_parameter_value().get<double>();
    accum_distance_thresh = node->get_parameter("accum_distance_thresh")
                                .get_parameter_value()
                                .get<double>();
    distance_from_last_edge_thresh =
        node->get_parameter("min_edge_interval").get_parameter_value().get<double>();

    fitness_score_max_range = node->get_parameter("fitness_score_max_range")
                                  .get_parameter_value()
                                  .get<double>();
    fitness_score_thresh =
        node->get_parameter("fitness_score_thresh").get_parameter_value().get<double>();

    keyframe_matching_threshold = node->get_parameter("keyframe_matching_threshold")
                                      .get_parameter_value()
                                      .get<double>();

    s_graphs::registration_params params;
    params = {
        node->get_parameter("registration_method")
            .get_parameter_value()
            .get<std::string>(),
        node->get_parameter("reg_num_threads").get_parameter_value().get<int>(),
        node->get_parameter("reg_transformation_epsilon")
            .get_parameter_value()
            .get<double>(),
        node->get_parameter("reg_maximum_iterations").get_parameter_value().get<int>(),
        node->get_parameter("reg_max_correspondence_distance")
            .get_parameter_value()
            .get<double>(),
        node->get_parameter("reg_correspondence_randomness")
            .get_parameter_value()
            .get<int>(),
        node->get_parameter("reg_resolution").get_parameter_value().get<double>(),
        node->get_parameter("reg_use_reciprocal_correspondences")
            .get_parameter_value()
            .get<bool>(),
        node->get_parameter("reg_max_optimizer_iterations")
            .get_parameter_value()
            .get<int>(),
        node->get_parameter("reg_nn_search_method")
            .get_parameter_value()
            .get<std::string>()};

    registration = select_registration_method(params);
    last_edge_accum_distance = 0.0;
  }

  /**
   * @brief Detect loops and add them to the pose graph
   *
   * @param keyframes
   *          Keyframes
   * @param new_keyframes
   *          Newly registered keyframes
   * @param covisibility_graph
   *          Pose graph
   * @return Loop vector
   */
  std::vector<Loop::Ptr> detect(const std::map<int, KeyFrame::Ptr>& keyframes,
                                const std::deque<KeyFrame::Ptr>& new_keyframes,
                                s_graphs::GraphSLAM& covisibility_graph) {
    std::vector<Loop::Ptr> detected_loops;
    for (const auto& new_keyframe : new_keyframes) {
      auto candidates = find_candidates_old(keyframes, new_keyframe);
      auto loop = matching(candidates, new_keyframe, covisibility_graph);
      if (loop) {
        detected_loops.push_back(loop);
      }
    }

    return detected_loops;
  }
  
  std::vector<Loop::Ptr> detect(const std::map<int, std::tuple<double, double, std::map<int, KeyFrame::Ptr>>>& room_keyframes,
                                const std::map<int, std::tuple<double, double, std::map<int, KeyFrame::Ptr>>>& newroom_keyframes,
                                const double roompart_length,
                                const std::string room_type,
                                s_graphs::GraphSLAM& covisibility_graph) {
    bool matched_w_room = false;
    std::map<int, KeyFrame::Ptr> finite_room_keyframes;
    int cnt1 = 0;  int cnt2= 0; 
    for (const auto & room_part: room_keyframes) {
      if (std::get<2>(room_part.second).empty()) continue;
      else cnt1++;
    }
    for (const auto & room_part: newroom_keyframes) {
      if (std::get<2>(room_part.second).empty()) continue;
      else cnt2++;
    }
    if ( cnt1==1 || cnt1==1 ) matched_w_room = true;

    std::vector<Loop::Ptr> detected_loops_cand;
    std::vector<double> detected_loops_scores_cand;
    for (const auto & room_part: room_keyframes) {
      if (std::get<2>(room_part.second).empty()) continue;

      for (const auto & new_room_part: newroom_keyframes) {
        if (std::get<2>(new_room_part.second).empty()) continue;
        if (matched_w_room) {
          for (const auto& new_keyframe : std::get<2>(new_room_part.second)) {
            auto candidates = find_candidates(std::get<2>(room_part.second), new_keyframe.second);
            double best_score = std::numeric_limits<double>::max();
            auto loop = matching(candidates, new_keyframe.second, covisibility_graph, best_score);
            bool is_new = true;
            if (loop) {
              for (int i=0; i<detected_loops_cand.size(); i++){
                if (detected_loops_cand[i]->key1 == loop->key1 || detected_loops_cand[i]->key2 == loop->key2 ){
                  is_new = false;
                  if (detected_loops_scores_cand[i] > best_score) {
                    detected_loops_cand[i] = loop;
                    detected_loops_scores_cand[i] = best_score;
                    break;
                  }
                }
              }
              if (is_new){
                detected_loops_cand.push_back(loop);
                detected_loops_scores_cand.push_back(best_score);
              }
            }
          }
        } else {
          double room_parts_dist = get_room_parts_dist(room_type, std::get<0>(room_part.second), std::get<1>(room_part.second), std::get<0>(new_room_part.second), std::get<1>(new_room_part.second));
          // std::cout<<"room_parts_dist "<<room_parts_dist<<std::endl;
          if (room_parts_dist > roompart_length) continue;

          for (const auto& new_keyframe : std::get<2>(new_room_part.second)) {
            auto candidates = find_candidates(std::get<2>(room_part.second), new_keyframe.second);
            double best_score = std::numeric_limits<double>::max();
            auto loop = matching(candidates, new_keyframe.second, covisibility_graph, best_score);
            bool is_new = true;
            if (loop) {
              for (int i=0; i<detected_loops_cand.size(); i++){
                if (detected_loops_cand[i]->key1 == loop->key1 || detected_loops_cand[i]->key2 == loop->key2 ){
                  is_new = false;
                  if (detected_loops_scores_cand[i] > best_score) {
                    detected_loops_cand[i] = loop;
                    detected_loops_scores_cand[i] = best_score;
                    break;
                  }
                }
              }
              if (is_new){
                detected_loops_cand.push_back(loop);
                detected_loops_scores_cand.push_back(best_score);
              }
            }
          }
        }
      }
    }
    std::vector<Loop::Ptr> detected_loops;
    int cnt = 0;
    double min_score = 100.0;
    int min_score_idx = -1;
    for (int i=0; i<detected_loops_scores_cand.size(); i++){
      if (detected_loops_scores_cand[i] < fitness_score_thresh) {
        cnt++;
        detected_loops.push_back(detected_loops_cand[i]);
      }
      if (detected_loops_scores_cand[i]<min_score){
        min_score = detected_loops_scores_cand[i];
        min_score_idx = i;
      }
    }
    if (detected_loops.size()>0) return detected_loops;
    else if (min_score_idx!=-1 && min_score<1.0) {
      std::cout<<"survived match "<<min_score<<std::endl;
      detected_loops.push_back(detected_loops_cand[min_score_idx]);
    }
    return detected_loops;
  }

  double get_room_parts_dist(const std::string room_type,
                                double x1, double y1, double x2, double y2){
    // if (room_type=="x inf room")
    //   return abs(y1-y2);
    // else if (room_type=="y inf room")
    //   return abs(x1-x2);
    // else return 0.0;
    return sqrt(pow(x1-x2,2)+pow(y1-y2,2));
  }
  // std::vector<Loop::Ptr> detect(const std::map<int, std::tuple<double, double, std::map<int, KeyFrame::Ptr>>>& room_keyframes,
  //                               const std::map<int, std::tuple<double, double, std::map<int, KeyFrame::Ptr>>>& newroom_keyframes,
  //                               const double roompart_length,
  //                               s_graphs::GraphSLAM& covisibility_graph) {
  //   std::vector<Loop::Ptr> detected_loops;
  //   std::vector<double> detected_loops_scores;
  //   for (const auto & room_part: room_keyframes) {
  //     for (const auto & new_room_part: newroom_keyframes) {
  //       if (std::get<2>(room_part.second).empty() || std::get<2>(new_room_part.second).empty()) continue;

  //       double room_parts_dist = get_room_parts_dist(std::get<0>(room_part.second), std::get<1>(room_part.second), std::get<0>(new_room_part.second), std::get<1>(new_room_part.second));
  //       if (room_parts_dist < roompart_length) {
  //         for (const auto& new_keyframe : std::get<2>(new_room_part.second)) {
  //           auto candidates = find_candidates(std::get<2>(room_part.second), new_keyframe.second);
  //           double best_score = std::numeric_limits<double>::max();
  //           auto loop = matching(candidates, new_keyframe.second, covisibility_graph, best_score);
  //           bool is_new = true;
  //           if (loop) {
  //             for (int i=0; i<detected_loops.size(); i++){
  //               if (detected_loops[i]->key1 == loop->key1 || detected_loops[i]->key2 == loop->key2 ){
  //                 is_new = false;
  //                 if (detected_loops_scores[i] > best_score) {
  //                   detected_loops[i] = loop;
  //                   detected_loops_scores[i] = best_score;
  //                   break;
  //                 }
  //               }
  //             }
  //             if (is_new){
  //               detected_loops.push_back(loop);
  //               detected_loops_scores.push_back(best_score);
  //             }
  //           }
  //         }
  //       }
  //     }
  //   }
  //   return detected_loops;
  // }


  std::vector<Loop::Ptr> detectWithAllKeyframes(
      const std::map<int, KeyFrame::Ptr>& keyframes,
      const std::vector<KeyFrame::Ptr>& new_keyframes,
      s_graphs::GraphSLAM& covisibility_graph) {
    std::vector<Loop::Ptr> detected_loops;
    for (const auto& new_keyframe : new_keyframes) {
      auto loop = matching(keyframes, new_keyframe, covisibility_graph, false);
      if (loop) {
        detected_loops.push_back(loop);
      }
    }

    return detected_loops;
  }

  bool matching(const KeyFrame::Ptr& keyframe,
                const KeyFrame::Ptr& prev_keyframe,
                Eigen::Matrix4f& relative_pose) {
    relative_pose.setIdentity();
    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());

    registration->setInputTarget(prev_keyframe->cloud);
    registration->setInputSource(keyframe->cloud);
    Eigen::Isometry3d prev_keyframe_estimate = prev_keyframe->node->estimate();
    prev_keyframe_estimate.linear() =
        Eigen::Quaterniond(prev_keyframe_estimate.linear())
            .normalized()
            .toRotationMatrix();
    Eigen::Isometry3d keyframe_estimate = keyframe->node->estimate();
    keyframe_estimate.linear() =
        Eigen::Quaterniond(keyframe_estimate.linear()).normalized().toRotationMatrix();

    Eigen::Matrix4f guess =
        (keyframe_estimate.inverse() * prev_keyframe_estimate).matrix().cast<float>();
    guess(2, 3) = 0.0;
    registration->align(*aligned, guess);

    double score = registration->getFitnessScore(fitness_score_max_range);

    if (!registration->hasConverged() || score > keyframe_matching_threshold) {
      return false;
    }

    relative_pose = registration->getFinalTransformation();
    return true;
  }

  /**
   * @brief
   *
   * @return Distance treshold
   */
  double get_distance_thresh() const { return distance_thresh; }

 private:
  /**
   * @brief Find loop candidates. A detected loop begins at one of #keyframes and ends
   * at #new_keyframe
   *
   * @param keyframes
   *          Candidate keyframes of loop start
   * @param new_keyframe
   *          Loop end keyframe
   * @return Loop candidates
   */
  std::map<int, KeyFrame::Ptr> find_candidates(
      const std::map<int, KeyFrame::Ptr>& keyframes,
      const KeyFrame::Ptr& new_keyframe) const {
    // too close to the last registered loop edge
    if (new_keyframe->accum_distance - last_edge_accum_distance <
        distance_from_last_edge_thresh) {
      return std::map<int, KeyFrame::Ptr>();
    }

    std::map<int, KeyFrame::Ptr> candidates;
    // candidates.reserve(32);

    for (const auto& k : keyframes) {
      if (abs(k.second->id() - new_keyframe->id())<15) continue;
      // traveled distance between keyframes is too small
      if (new_keyframe->accum_distance - k.second->accum_distance <
          accum_distance_thresh) {
        continue;
      }

      const auto& pos1 = k.second->node->estimate().translation();
      const auto& pos2 = new_keyframe->node->estimate().translation();

      // estimated distance between keyframes is too small
      double dist = (pos1.head<2>() - pos2.head<2>()).norm();
      if (dist > distance_thresh) {
        continue;
      }

      candidates.insert({k.first, k.second});
    }

    return candidates;
  }

  std::map<int, KeyFrame::Ptr> find_candidates_old(
      const std::map<int, KeyFrame::Ptr>& keyframes,
      const KeyFrame::Ptr& new_keyframe) const {
    // too close to the last registered loop edge
    if (new_keyframe->accum_distance - last_edge_accum_distance <
        distance_from_last_edge_thresh) {
      return std::map<int, KeyFrame::Ptr>();
    }

    std::map<int, KeyFrame::Ptr> candidates;
    // candidates.reserve(32);

    for (const auto& k : keyframes) {
      // traveled distance between keyframes is too small
      if (new_keyframe->accum_distance - k.second->accum_distance <
          accum_distance_thresh) {
        continue;
      }

      const auto& pos1 = k.second->node->estimate().translation();
      const auto& pos2 = new_keyframe->node->estimate().translation();

      // estimated distance between keyframes is too small
      double dist = (pos1.head<2>() - pos2.head<2>()).norm();
      if (dist > distance_thresh) {
        continue;
      }

      candidates.insert({k.first, k.second});
    }

    return candidates;
  }

  /**
   * @brief To validate a loop candidate this function applies a scan matching between
   * keyframes consisting the loop. If they are matched well, the loop is added to the
   * pose graph
   *
   * @param candidate_keyframes
   *          candidate keyframes of loop start
   * @param new_keyframe
   *          loop end keyframe
   * @param covisibility_graph
   *          graph slam
   * @return Loop pointer
   */
  Loop::Ptr matching(const std::map<int, KeyFrame::Ptr>& candidate_keyframes,
                     const KeyFrame::Ptr& new_keyframe,
                     s_graphs::GraphSLAM& covisibility_graph,
                     bool use_prior = true) {
    if (candidate_keyframes.empty() || new_keyframe->cloud->points.empty()) {
      return nullptr;
    }

    registration->setInputTarget(new_keyframe->cloud);

    double best_score = std::numeric_limits<double>::max();
    KeyFrame::Ptr best_matched;
    Eigen::Matrix4f relative_pose;

    // std::cout << std::endl;
    // std::cout << "--- loop detection ---" << std::endl;
    // std::cout << "num_candidates: " << candidate_keyframes.size() << std::endl;
    // std::cout << "matching" << std::flush;
    auto t1 = rclcpp::Clock{}.now();

    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    for (const auto& candidate : candidate_keyframes) {
      if (candidate.second->cloud->points.empty()) continue;
      registration->setInputSource(candidate.second->cloud);
      Eigen::Isometry3d new_keyframe_estimate = new_keyframe->node->estimate();
      new_keyframe_estimate.linear() =
          Eigen::Quaterniond(new_keyframe_estimate.linear())
              .normalized()
              .toRotationMatrix();
      Eigen::Isometry3d candidate_estimate = candidate.second->node->estimate();
      candidate_estimate.linear() = Eigen::Quaterniond(candidate_estimate.linear())
                                        .normalized()
                                        .toRotationMatrix();

      if (use_prior) {
        Eigen::Matrix4f guess = (new_keyframe_estimate.inverse() * candidate_estimate)
                                    .matrix()
                                    .cast<float>();
        guess(2, 3) = 0.0;
        registration->align(*aligned, guess);
      } else {
        Eigen::Matrix4f guess;
        guess << 1, 0, 0, 0, 0, 1, 0, 10, 0, 0, 1, 0, 0, 0, 0, 1;
        // std::cout << "Using our guess";
        registration->align(*aligned, guess);
        // registration->align(*aligned);
      }
      // std::cout << "." << std::flush;

      double score = registration->getFitnessScore(fitness_score_max_range);
      if (!registration->hasConverged() || score > best_score) {
        continue;
      }

      best_score = score;
      best_matched = candidate.second;
      relative_pose = registration->getFinalTransformation();
    }

    auto t2 = rclcpp::Clock{}.now();
    // std::cout << " done" << std::endl;
    // std::cout << "best_score: " << boost::format("%.3f") % best_score
    //           << "    time: " << boost::format("%.3f") % (t2 - t1).seconds() <<
    //           "[sec]"
    //           << std::endl;

    if (best_score > 1.0) {
      std::cout << "loop not found... BEST SCORE:" << best_score << std::endl;
      return nullptr;
    } else {
      // std::cout << "loop found:" << best_score << std::endl;
    }

    // std::cout << "loop found!!" << std::endl;
    // std::cout
    //     << "relpose: " << relative_pose.block<3, 1>(0, 3) << " - "
    //     << Eigen::Quaternionf(relative_pose.block<3, 3>(0, 0)).coeffs().transpose()
    //     << std::endl;

    last_edge_accum_distance = new_keyframe->accum_distance;

    return std::make_shared<Loop>(new_keyframe, best_matched, relative_pose);
  }
  Loop::Ptr matching(const std::map<int, KeyFrame::Ptr>& candidate_keyframes,
                     const KeyFrame::Ptr& new_keyframe,
                     s_graphs::GraphSLAM& covisibility_graph,
                     double &best_score,
                     bool use_prior = true) {
    if (candidate_keyframes.empty() || new_keyframe->cloud->points.empty()) {
      return nullptr;
    }

    registration->setInputTarget(new_keyframe->cloud);

    best_score = std::numeric_limits<double>::max();
    KeyFrame::Ptr best_matched;
    Eigen::Matrix4f relative_pose;

    // std::cout << std::endl;
    // std::cout << "--- loop detection ---" << std::endl;
    // std::cout << "num_candidates: " << candidate_keyframes.size() << std::endl;
    // std::cout << "matching" << std::flush;
    auto t1 = rclcpp::Clock{}.now();

    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    for (const auto& candidate : candidate_keyframes) {
      if (candidate.second->cloud->points.empty()) continue;
      registration->setInputSource(candidate.second->cloud);
      Eigen::Isometry3d new_keyframe_estimate = new_keyframe->node->estimate();
      new_keyframe_estimate.linear() =
          Eigen::Quaterniond(new_keyframe_estimate.linear())
              .normalized()
              .toRotationMatrix();
      Eigen::Isometry3d candidate_estimate = candidate.second->node->estimate();
      candidate_estimate.linear() = Eigen::Quaterniond(candidate_estimate.linear())
                                        .normalized()
                                        .toRotationMatrix();

      if (use_prior) {
        Eigen::Matrix4f guess = (new_keyframe_estimate.inverse() * candidate_estimate)
                                    .matrix()
                                    .cast<float>();
        guess(2, 3) = 0.0;
        registration->align(*aligned, guess);
      } else {
        Eigen::Matrix4f guess;
        guess << 1, 0, 0, 0, 0, 1, 0, 10, 0, 0, 1, 0, 0, 0, 0, 1;
        // std::cout << "Using our guess";
        registration->align(*aligned, guess);
        // registration->align(*aligned);
      }
      // std::cout << "." << std::flush;

      double score = registration->getFitnessScore(fitness_score_max_range);
      if (!registration->hasConverged() || score > best_score) {
        continue;
      }

      best_score = score;
      best_matched = candidate.second;
      relative_pose = registration->getFinalTransformation();
    }

    auto t2 = rclcpp::Clock{}.now();
    // std::cout << " done" << std::endl;
    // std::cout << "best_score: " << boost::format("%.3f") % best_score
    //           << "    time: " << boost::format("%.3f") % (t2 - t1).seconds() <<
    //           "[sec]"
    //           << std::endl;

    if (best_score > 1.0) {
      std::cout << "loop not found... BEST SCORE:" << best_score << std::endl;
      return nullptr;
    } else {
      // std::cout << "loop found:" << best_score << std::endl;
    }

    // std::cout << "loop found!!" << std::endl;
    // std::cout
    //     << "relpose: " << relative_pose.block<3, 1>(0, 3) << " - "
    //     << Eigen::Quaternionf(relative_pose.block<3, 3>(0, 0)).coeffs().transpose()
    //     << std::endl;

    last_edge_accum_distance = new_keyframe->accum_distance;

    return std::make_shared<Loop>(new_keyframe, best_matched, relative_pose);
  }

 private:
  double distance_thresh;  // estimated distance between keyframes consisting a loop
                           // must be less than this distance
  double accum_distance_thresh;           // traveled distance between ...
  double distance_from_last_edge_thresh;  // a new loop edge must far from the last one
                                          // at least this distance

  double fitness_score_max_range;  // maximum allowable distance between corresponding
                                   // points
  double fitness_score_thresh;     // threshold for scan matching
  double keyframe_matching_threshold;  // threshold for disconnected keyframes

  double last_edge_accum_distance;

  pcl::Registration<PointT, PointT>::Ptr registration;
};

}  // namespace s_graphs

#endif  // LOOP_DETECTOR_HPP
