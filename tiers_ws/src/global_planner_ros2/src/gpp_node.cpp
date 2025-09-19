#include <rclcpp/rclcpp.hpp>
#include "gpp/pathplan.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
	auto n = rclcpp::Node::make_shared("lm_pathplan");
  saslam::pathplan pathplan_class = saslam::pathplan();



  // register pub
  pathplan_class.m_pub_nextWaypt = n->create_publisher<geometry_msgs::msg::Point>("/gpp/nextWaypt",100);
  pathplan_class.m_pub_wayPt = n->create_publisher<sensor_msgs::msg::PointCloud2>("/gpp/wayPoint",100);
  pathplan_class.m_pub_splinedWayPt = n->create_publisher<nav_msgs::msg::Path>("/gpp/splinedWayPoint",100);

  pathplan_class.m_pub_base_grid = n->create_publisher<nav_msgs::msg::OccupancyGrid>("/gpp/base/grid",100);
  pathplan_class.m_pub_base_marker = n->create_publisher<visualization_msgs::msg::Marker>("/gpp/base/marker",100);
  pathplan_class.m_pub_customMap = n->create_publisher<mgs04_base_msgs::msg::Map>("/custom/map/check", 100);


  // register sub
  // auto sub1 = n->create_subscription<nav_msgs::msg::Odometry>(
  //               "/odometry", 
  //               rclcpp::QoS(rclcpp::KeepLast(10)), 
  //               std::bind(&saslam::pathplan::callback_crtState, &pathplan_class, std::placeholders::_1)
  //               );//EDITED_MICROSWARM
  // auto sub2 = n->create_subscription<geometry_msgs::msg::PoseStamped>(
  //               "/goal_pose", 
  //               rclcpp::QoS(rclcpp::KeepLast(10)), 
  //               std::bind(&saslam::pathplan::callback_goal, &pathplan_class, std::placeholders::_1)
  //               );
  auto sub3 = n->create_subscription<geometry_msgs::msg::PointStamped>(
                "/gpp/waypoint", 
                rclcpp::QoS(rclcpp::KeepLast(10)), 
                std::bind(&saslam::pathplan::callback_waypt, &pathplan_class, std::placeholders::_1)
                );

  auto sub4 = n->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/grid_map", 
                rclcpp::QoS(rclcpp::KeepLast(10)), 
                std::bind(&saslam::pathplan::callback_2dmap, &pathplan_class, std::placeholders::_1)
                );//EDITED_MICROSWARM

  // Subscribe to the 2D Pose Estimate (PoseWithCovarianceStamped)
  auto pose_with_covariance_sub_ = n->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            std::bind(&saslam::pathplan::callback_initialPose, &pathplan_class, std::placeholders::_1));
  // Subscribe to the 2D Goal Pose (PoseStamped)
  auto pose_goal_sub_ = n->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, 
            std::bind(&saslam::pathplan::callback_goalPose, &pathplan_class, std::placeholders::_1));

  pathplan_class.getParam(n);
  // pathplan_class.pubBase(n);


  // n->create_wall_timer(
  //         std::chrono::seconds(0.1),
  //         std::bind(pathplan_class.spinTimer, n));



  rclcpp::spin(n);
  return 0;
}
