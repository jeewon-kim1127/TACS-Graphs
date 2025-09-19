#ifndef PATHPLAN_H
#define PATHPLAN_H


#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <visualization_msgs/msg/marker.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cmath>

#include <gazebo_msgs/msg/model_states.hpp>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

// custom msgs are not used.. why ?!?!?!!? 
// #include "saslam_msg/msg/s_amcl.hpp" 
#include "planner/ShortestPP.h"

#include "mgs04_base_msgs/msg/cmd_vel.hpp"
#include "mgs04_base_msgs/msg/map.hpp"
#include <boost/format.hpp>


// #include "unavlib/convt.h"
// #include "unavlib/others.h"


namespace saslam
{
class pathplan
{
    struct object{
        int classIdx;
        int groupIdx;
        geometry_msgs::msg::Point location;
        std::vector<int> classRatio;
    };

    struct subPath{
        std::vector<Eigen::Vector2f> splinedPath;
        std::vector<Eigen::Vector2f> rawPath;
    };
    struct occumapCv{
        cv::Mat cvMapForPlan;
        cv::Mat cvMapForCheck;
        double resolution;
        Eigen::MatrixXf m_T_global2gridmap;
    };
    struct robotParam{
        double robotSize = 2.0;
        double robotMargin = 0.4;
    };

// private:
public:
    bool save_time = true;
	// auto m_nh; 
    rclcpp::Node::SharedPtr m_nh_priv;
    rclcpp::TimerBase::SharedPtr timer1;

    Eigen::MatrixXf m_curPose;
    cv::Point curPoseOnGridCV;
    Eigen::MatrixXf curPoseOnGrid;
    cv::Mat cvMapForPlan;
    
    std::vector<std::pair<geometry_msgs::msg::Point,subPath>> m_waypts;

    std::vector<std::string> m_classes;
    std::vector<Eigen::Vector3i> m_colors;
    std::vector<object> m_objects;
    std::vector<std::vector<int>> m_groups;
    std::vector<int> m_groups_class;
    std::vector<geometry_msgs::msg::Point> m_groups_pose;

    std::string m_param_mapdir = "/home/jimi/.ros";
    nav_msgs::msg::OccupancyGrid m_occumap;
    occumapCv m_occumapCv;
    robotParam m_robotParam;
    bool m_useObject = false;
    double m_successDistance = 0.1;
    double m_safeAreaSide = 0.2;
    double m_safeAreaFrontMin = 0.4;
    double m_safeAreaFront = 0.7;

    int map_callback_count = 0;

    bool m_flag_stop;
    bool m_flag_init = false;
    void getParam(rclcpp::Node::SharedPtr n);

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_pub_base_grid;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_pub_base_marker;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr m_pub_nextWaypt;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pub_wayPt;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_pub_splinedWayPt;
    rclcpp::Publisher<mgs04_base_msgs::msg::Map>::SharedPtr m_pub_customMap;

//    void callback_crtState(saslam::sa_mcl::SharedPtr msg);
//    void callback_crtState(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void callback_initialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void callback_goalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void callback_crtState(nav_msgs::msg::Odometry::SharedPtr msg);//EDITED_MICROSWARM
    void callback_goal(geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void callback_waypt(geometry_msgs::msg::PointStamped::SharedPtr msg);
    void spinTimer(std::shared_ptr<rclcpp::Node> n);

    /* Newly added */
    void callback_2dmap(nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    void pubBase(std::shared_ptr<rclcpp::Node> n);
    void procMap();

    subPath pathplanning(cv::Mat map, cv::Point src, cv::Point dst);
    float distancePoint(Eigen::MatrixXf pose,Eigen::Vector2f wayPt);
    void clearWaypt();

    pathplan();
    ~pathplan();



    // from unavlib
    template<typename T>
    sensor_msgs::msg::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, std::string frame_id = "world")
    {
        sensor_msgs::msg::PointCloud2 cloud_ROS;
        pcl::toROSMsg(cloud, cloud_ROS);
        cloud_ROS.header.frame_id = frame_id;
        return cloud_ROS;
    }

    Eigen::Matrix4f geoPose2eigen(geometry_msgs::msg::Pose geoPose)
    {
        Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
        Eigen::Quaterniond q(geoPose.orientation.w, geoPose.orientation.x, geoPose.orientation.y, geoPose.orientation.z);
        
        // zinuok: this may be cause of error (differs from the original code)
        Eigen::Matrix3d m = q.toRotationMatrix(); 
        result(0,0) = m(0,0);
        result(0,1) = m(0,1);
        result(0,2) = m(0,2);
        result(1,0) = m(1,0);
        result(1,1) = m(1,1);
        result(1,2) = m(1,2);
        result(2,0) = m(2,0);
        result(2,1) = m(2,1);
        result(2,2) = m(2,2);
        result(3,3) = 1;

        result(0,3) = geoPose.position.x;
        result(1,3) = geoPose.position.y;
        result(2,3) = geoPose.position.z;

        return result;
    }


    Eigen::MatrixXf geoPoint2eigen(geometry_msgs::msg::Point geoPoint)
    {
      Eigen::MatrixXf result(4,1);
      result << geoPoint.x, geoPoint.y, geoPoint.z, 1;
      return result;
    }

    cv::Vec3b heightcolor(double h)
    {
        if(h > 1) h = 1;
        if(h < 0) h = 0;

        h = h * 0.667;


        double color_R;
        double color_G;
        double color_B;
        // blend over HSV-values (more colors)

        double s = 1.0;
        double v = 1.0;

        h -= floor(h);
        h *= 6;
        int i;
        double m, n, f;

        i = floor(h);
        f = h - i;
        if (!(i & 1))
        f = 1 - f; // if i is even
        m = v * (1 - s);
        n = v * (1 - s * f);

        switch (i) {
        case 6:
        case 0:
        color_R = v; color_G = n; color_B = m;
        break;
        case 1:
        color_R = n; color_G = v; color_B = m;
        break;
        case 2:
        color_R = m; color_G = v; color_B = n;
        break;
        case 3:
        color_R = m; color_G = n; color_B = v;
        break;
        case 4:
        color_R = n; color_G = m; color_B = v;
        break;
        case 5:
        color_R = v; color_G = m; color_B = n;
        break;
        default:
        color_R = 1; color_G = 0.5; color_B = 0.5;
        break;
        }
        cv::Vec3b color;
        color[0] = color_R * 255;
        color[1] = color_G * 255;
        color[2] = color_B * 255;
        return color;
    }


    nav_msgs::msg::OccupancyGrid cvimg2occumap(cv::Mat cvimg, float resolution,cv::Point origin_cvpt)
    {
      nav_msgs::msg::OccupancyGrid m_gridmap;
      m_gridmap.info.resolution = resolution;
      geometry_msgs::msg::Pose origin;
      origin.position.x = -origin_cvpt.x * resolution; origin.position.y = -origin_cvpt.y * resolution;
      origin.orientation.w = 1;
      m_gridmap.info.origin = origin;
      m_gridmap.info.width = cvimg.size().width;
      m_gridmap.info.height = cvimg.size().height;
      //ROS_INFO("[CV 2 OCCUGRID CONVERSION] PUT CV DATA");
      for(int i=0;i<m_gridmap.info.width*m_gridmap.info.height;i++) m_gridmap.data.push_back(-1);
      //ROS_INFO("[CV 2 OCCUGRID CONVERSION] GRID SIZE : %d",m_gridmap.data.size());
      //ROS_INFO("[CV 2 OCCUGRID CONVERSION] GRID ORIGIN : [%d,%d]",origin_cvpt.x,origin_cvpt.y);

      for(int y = 0; y < cvimg.size().height; y++)
      {
        for(int x = 0; x < cvimg.size().width; x++)
        {
          int tmpdata = cvimg.at<unsigned char>(y,x);
          int ttmpdata = -1; //Unknown
          if(tmpdata >= 150) //free
          {
            ttmpdata = (tmpdata - 250) / -2;
            if(ttmpdata < 0)
              ttmpdata = 0;
          }
          else if(tmpdata <= 98)
          {
            ttmpdata = (tmpdata - 200) / -2;
            if(ttmpdata > 100)
              ttmpdata = 100;
          }
          m_gridmap.data.at(x + m_gridmap.info.width * y) = ttmpdata;
        }
      }
      return m_gridmap;
    }


    cv::Mat occumap2cvimg( nav_msgs::msg::OccupancyGrid occumap)
    {
      // unkown = -1 -> 99~149, free : 0:50 ->250:150, occupied :51:100 -> 0:98
      double resolution = occumap.info.resolution;
      cv::Point origin_cvpt(-occumap.info.origin.position.x / resolution,
                            -occumap.info.origin.position.y / resolution);
      cv::Size img_size;
      cv::Mat cvimg = cv::Mat::zeros( occumap.info.height,occumap.info.width,CV_8UC1);
      //ROS_INFO("[OCCUGRID 2 CVT CONVERSION] GRID SIZE : %d",occumap.data.size());
      //ROS_INFO("[OCCUGRID 2 CVT CONVERSION] CV ORIGIN : [%d,%d]",origin_cvpt.x,origin_cvpt.y);
      for(int pt = 0;pt < occumap.data.size();pt++)
      {
        int pt_y = pt / occumap.info.width;
        int pt_x = pt % occumap.info.width;
        int value = occumap.data.at(pt);
        unsigned char img_value;

        // if(value == -1) img_value = 120;
        // if (value ==0 ) img_value = 255;
        // else if (value == 255) img_value = 0;

        if (value == -1) img_value = 120;
        else if (value == 0) img_value = 0;
        else if (value == 100) img_value = 255;
        cvimg.at<unsigned char>(pt_y,pt_x) = img_value;
      }
      return cvimg;
    }

    bool loadOccupancymap(std::string filepath, nav_msgs::msg::OccupancyGrid& gridmap_out)
    {
      std::stringstream strm_png;
      strm_png << filepath <<".png";
      std::stringstream strm_info;
      strm_info << filepath << ".csv";
      cv::Mat occumat = cv::imread(strm_png.str(),cv::IMREAD_GRAYSCALE);
      std::ifstream fileload(strm_info.str().c_str());
      float resolution,origin_x,origin_y;
      std::vector<std::string>   result;
      std::string line;
      if(!fileload.is_open())
      {
        std::cout << "Warning : Canot open occupancy map" << std::endl;
        return false;
      }
      while(std::getline(fileload,line)) result.push_back(line);
      if(result.size()!=3)
      {
        std::cout << "Warning : Canot open occupancy map (arguments)" << std::endl;
        return false;
      }
      resolution = std::atof(result.at(0).c_str());
      origin_x = std::atof(result.at(1).c_str());
      origin_y = std::atof(result.at(2).c_str());
      gridmap_out = cvimg2occumap(occumat, resolution,cv::Point(- origin_x / resolution, - origin_y / resolution));
      gridmap_out.header.frame_id = "world";

      return true;
    }



};
}

#endif

