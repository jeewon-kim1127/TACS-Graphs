#include <chrono>
#include <memory>
#include <string>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>

typedef pcl::PointXYZ PointType;

class LidarsCalibrater : public rclcpp::Node
{
public:
    LidarsCalibrater() : Node("lidar_calibrate")
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        sub_os0_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/os0_cloud_node/points", 10, std::bind(&LidarsCalibrater::os0_cloud_handler, this, std::placeholders::_1));

        // sub_os0_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        //     "/os_cloud_node/imu", 10, std::bind(&LidarsCalibrater::os0_imu_handler, this, std::placeholders::_1));

        pub_os0_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/a_os0", 1);

        os0_tf_initd_ = false;
        os0_received_ = false;
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_os0_;
    // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_os0_imu_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_os0_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    Eigen::Matrix4f os0_tf_matrix_;
    bool os0_tf_initd_;
    pcl::PointCloud<PointType> os0_cloud_;
    bool os0_received_;
    sensor_msgs::msg::Imu::SharedPtr latest_imu_msg_;

    void os0_cloud_handler(const sensor_msgs::msg::PointCloud2::SharedPtr pointCloudIn)
    {
        pcl::PointCloud<PointType> full_cloud_in;
        pcl::fromROSMsg(*pointCloudIn, full_cloud_in);
        pcl_conversions::toPCL(pointCloudIn->header, full_cloud_in.header);

        // Initial transformation matrix for OS0
        Eigen::AngleAxisf init_rot_x(0.0, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf init_rot_y(0.0, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf init_rot_z(M_PI / 4, Eigen::Vector3f::UnitZ());
        Eigen::Translation3f init_trans(0.0, 0.0, 0.0);
        Eigen::Matrix4f init_tf = (init_trans * init_rot_z * init_rot_y * init_rot_x).matrix();

        if (!os0_tf_initd_)
        {
            // Output the transformation to the console
            Eigen::Matrix3f rot_matrix = init_tf.block(0, 0, 3, 3);
            Eigen::Vector3f trans_vector = init_tf.block(0, 3, 3, 1);

            std::cout << "OS0 -> base_link " << trans_vector.transpose()
                      << " " << rot_matrix.eulerAngles(2, 1, 0).transpose() << " /"
                      << "os0_sensor"
                      << " /"
                      << "base_link"
                      << " 10" << std::endl;

            os0_tf_initd_ = true;
        }

        // Apply the transformation to the point cloud
        pcl::PointCloud<PointType> out_cloud;
        pcl::transformPointCloud(full_cloud_in, full_cloud_in, init_tf);

        os0_cloud_.clear();
        os0_cloud_ += full_cloud_in;
        os0_received_ = true;

        // Broadcast the transform
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = "base_link";
        transform_stamped.child_frame_id = "os0_sensor";

        transform_stamped.transform.translation.x = init_tf(0, 3);
        transform_stamped.transform.translation.y = init_tf(1, 3);
        transform_stamped.transform.translation.z = init_tf(2, 3);

        Eigen::Quaternionf q(init_tf.block<3, 3>(0, 0));
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        // tf_broadcaster_->sendTransform(transform_stamped);

        // Publish the transformed point cloud
        sensor_msgs::msg::PointCloud2 os0_msg;
        pcl::toROSMsg(os0_cloud_, os0_msg);
        os0_msg.header.stamp = this->get_clock()->now();
        os0_msg.header.frame_id = "base_link";
        pub_os0_->publish(os0_msg);
    }

    // void os0_imu_handler(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    // {
    //     // Store the latest IMU message
    //     latest_imu_msg_ = imu_msg;

    //     // Optionally broadcast a transform based on the IMU data
    //     geometry_msgs::msg::TransformStamped imu_transform;
    //     imu_transform.header.stamp = imu_msg->header.stamp;
    //     imu_transform.header.frame_id = "base_link";
    //     imu_transform.child_frame_id = "os0_imu";

    //     // Use the IMU's orientation directly
    //     imu_transform.transform.rotation = imu_msg->orientation;

    //     // Assuming zero translation for the IMU (if you know the IMU's offset, you can add it here)
    //     imu_transform.transform.translation.x = 0.0;
    //     imu_transform.transform.translation.y = 0.0;
    //     imu_transform.transform.translation.z = 0.0;

    //     // Broadcast the IMU transform
    //     tf_broadcaster_->sendTransform(imu_transform);
    // }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarsCalibrater>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
