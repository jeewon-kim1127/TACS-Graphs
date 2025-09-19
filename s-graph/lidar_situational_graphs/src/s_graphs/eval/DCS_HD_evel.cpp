// #include <rclcpp/rclcpp.hpp>
// #include <fstream>
// #include <sstream>
// #include <vector>
// #include <cmath>
// #include <algorithm>

// struct Room {
//     std::string room_id;
//     double center_x;
//     double center_y;
//     double length_x;
//     double length_y;
// };

// class SegmentationEvaluator : public rclcpp::Node {
// public:
//     SegmentationEvaluator() : Node("segmentation_evaluator"), center_distance_threshold_(0.5) {
//         // this->declare_parameter<std::string>("folder_path", "");
//         // this->declare_parameter<double>("center_distance_threshold", 0.5);
        
//         // center_distance_threshold_ = this->get_parameter("center_distance_threshold").as_double();
//         // std::string folder_path = this->get_parameter("folder_path").as_string();
        
//         std::string data_dir = "/root/workspaces/data/tiers/old/true/";
//         for (int i=1; i<6; i++){
//             std::string csv_file_path = data_dir+"iteration_"+to_string(i)+"_consistency.csv";
//             if (loadRoomDataFromCSV(csv_file_path)) {
//             } else {
//                 RCLCPP_ERROR(this->get_logger(), "%s is empty.", csv_file_path);
//             }
//         }
//         evaluateConsistency();
//     }

// private:
//     std::vector<std::vector<Room>> trials_; // Stores room data for each trial
//     double center_distance_threshold_; // Threshold to consider rooms as the same

//     bool loadRoomDataFromCSV(const std::string &file_path) {
//         std::ifstream file(file_path);
//         if (!file.is_open()) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path.c_str());
//             return false;
//         }

//         std::string line;
//         size_t current_trial = 0;
//         // trials_.emplace_back(); // First trial entry
//         std::vector<Room> single_trial;

//         while (std::getline(file, line)) {
//             std::stringstream ss(line);
//             Room room;
//             std::string token;

//             // Assuming the CSV structure: center_x, center_y, length_x, length_y
//             std::getline(ss, room.room_id, ',');
//             std::getline(ss, token, ',');
//             room.center_x = std::stod(token);
//             std::getline(ss, token, ',');
//             room.center_y = std::stod(token);
//             std::getline(ss, token, ',');
//             room.length_x = std::stod(token);
//             std::getline(ss, token, ',');
//             room.length_y = std::stod(token);
//             // std::getline(ss, token, ',');
//             // size_t trial_number = std::stoul(token);

//             // if (trial_number != current_trial) {
//             //     trials_.emplace_back();
//             //     current_trial = trial_number;
//             // }

//             single_trial.push_back(room);
//         }
//         trials_.push_back(single_trial);
        
//         file.close();
//         return true;
//     }

//     void evaluateConsistency() {
//         double dice_sum = 0.0;
//         double hausdorff_sum = 0.0;
//         int comparisons = 0;

//         for (size_t i = 0; i < trials_.size(); ++i) {
//             for (size_t j = i + 1; j < trials_.size(); ++j) {
//                 for (const auto &room1 : trials_[i]) {
//                     for (const auto &room2 : trials_[j]) {
//                         double dcs = computeDiceCoefficient(room1, room2);
//                         if (dcs>max_dcs){
//                             max_dcs = dcs;
//                             max_matched = room2;
//                         }
//                     }
//                     std::cout<<room1.room_id<<" matched with "<<max_matched.room_id<<std::endl;
//                     dice_sum += max_dcs;
//                     // hausdorff_sum += computeHausdorffDistance(room1, room2);
//                     comparisons++;
//                 }
//             }
//         }

//         if (comparisons > 0) {
//             double avg_dice = dice_sum / comparisons;
//             // double avg_hausdorff = hausdorff_sum / comparisons;

//             RCLCPP_INFO(this->get_logger(), "Average Dice Coefficient: %f", avg_dice);
//             RCLCPP_INFO(this->get_logger(), "Average Hausdorff Distance: %f", avg_hausdorff);
//         } else {
//             RCLCPP_WARN(this->get_logger(), "No matching rooms found across trials.");
//         }
//     }

//     bool areSameRoom(const Room &room1, const Room &room2) {
//         double distance = std::sqrt(std::pow(room1.center_x - room2.center_x, 2.0) +
//                                     std::pow(room1.center_y - room2.center_y, 2.0));
//         return distance < center_distance_threshold_;
//     }

//     double computeDiceCoefficient(const Room &room1, const Room &room2) {
//         // Compute overlap area
//         double x_overlap = std::max(0.0, std::min(room1.center_x + room1.length_x / 2.0, room2.center_x + room2.length_x / 2.0) -
//                                          std::max(room1.center_x - room1.length_x / 2.0, room2.center_x - room2.length_x / 2.0));
//         double y_overlap = std::max(0.0, std::min(room1.center_y + room1.length_y / 2.0, room2.center_y + room2.length_y / 2.0) -
//                                          std::max(room1.center_y - room1.length_y / 2.0, room2.center_y - room2.length_y / 2.0));

//         if (x_overlap==0 || y_overlap) return 0.0;

//         double intersection_area = x_overlap * y_overlap;
//         double area1 = room1.length_x * room1.length_y;
//         double area2 = room2.length_x * room2.length_y;

//         double dice = (2.0 * intersection_area) / (area1 + area2);
//         return dice;
//     }

//     double computeHausdorffDistance(const Room &room1, const Room &room2) {
//         // Calculate corner points of each room
//         std::vector<std::pair<double, double>> corners1 = getCorners(room1);
//         std::vector<std::pair<double, double>> corners2 = getCorners(room2);

//         double max_dist_1_to_2 = getMaxMinDistance(corners1, corners2);
//         double max_dist_2_to_1 = getMaxMinDistance(corners2, corners1);

//         return std::max(max_dist_1_to_2, max_dist_2_to_1);
//     }

//     std::vector<std::pair<double, double>> getCorners(const Room &room) {
//         double half_length_x = room.length_x / 2.0;
//         double half_length_y = room.length_y / 2.0;

//         std::vector<std::pair<double, double>> corners;
//         corners.emplace_back(room.center_x - half_length_x, room.center_y - half_length_y);
//         corners.emplace_back(room.center_x - half_length_x, room.center_y + half_length_y);
//         corners.emplace_back(room.center_x + half_length_x, room.center_y - half_length_y);
//         corners.emplace_back(room.center_x + half_length_x, room.center_y + half_length_y);

//         return corners;
//     }

//     double getMaxMinDistance(const std::vector<std::pair<double, double>> &corners1,
//                              const std::vector<std::pair<double, double>> &corners2) {
//         double max_min_dist = 0.0;

//         for (const auto &corner1 : corners1) {
//             double min_dist = std::numeric_limits<double>::max();
//             for (const auto &corner2 : corners2) {
//                 double dist = std::sqrt(std::pow(corner1.first - corner2.first, 2.0) + std::pow(corner1.second - corner2.second, 2.0));
//                 min_dist = std::min(min_dist, dist);
//             }
//             max_min_dist = std::max(max_min_dist, min_dist);
//         }

//         return max_min_dist;
//     }
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<SegmentationEvaluator>());
//     rclcpp::shutdown();
//     return 0;
// }
