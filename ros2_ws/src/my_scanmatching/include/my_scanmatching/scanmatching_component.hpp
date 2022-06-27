#ifndef AUTOBIN__SM_COMPONENT_HPP_
#define AUTOBIN__SM_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define A_SMC_EXPORT __attribute__ ((dllexport))
    #define A_SMC_IMPORT __attribute__ ((dllimport))
  #else
    #define A_SMC_EXPORT __declspec(dllexport)
    #define A_SMC_IMPORT __declspec(dllimport)
  #endif
  #ifdef A_SMC_BUILDING_DLL
    #define A_SMC_PUBLIC A_SMC_EXPORT
  #else
    #define A_SMC_PUBLIC A_SMC_IMPORT
  #endif
  #define A_SMC_PUBLIC_TYPE A_SMC_PUBLIC
  #define A_SMC_LOCAL
#else
  #define A_SMC_EXPORT __attribute__ ((visibility("default")))
  #define A_SMC_IMPORT
  #if __GNUC__ >= 4
    #define A_SMC_PUBLIC __attribute__ ((visibility("default")))
    #define A_SMC_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define A_SMC_PUBLIC
    #define A_SMC_LOCAL
  #endif
  #define A_SMC_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <my_scanmatching/my_scanmatching_ekf.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rclcpp_components/register_node_macro.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <string>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <pclomp/ndt_omp.h>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp.h>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>
#include <pclomp/gicp_omp.h>
#include <pclomp/gicp_omp_impl.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <mutex>
#include <thread>
#include <future>
#include <iostream>

namespace autobin
{
    class ScanmatchingComponent : public rclcpp::Node
    {
        public:
            A_SMC_PUBLIC
            explicit ScanmatchingComponent(const rclcpp::NodeOptions & options);

        private:
            rclcpp::Clock clock;
            tf2_ros::Buffer tfbuffer;
            tf2_ros::TransformListener listener;
            tf2_ros::TransformBroadcaster broadcaster;

            std::string global_frame_id_;
            std::string robot_frame_id_;
            std::string lidar_frame_id_;
            std::string cloud_topic_;
            std::string odom_topic_;
            std::string registration_method_;

            double ndt_resolution_;
            int ndt_num_threads_;
            double gicp_corr_dist_threshold_;

            //registration
            pcl::Registration <pcl::PointXYZI, pcl::PointXYZI>::Ptr registration;

            //subcriber
            rclcpp::Subscription <nav_msgs::msg::Odometry>::SharedPtr sub_odom_pose;

            rclcpp::Subscription <sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_input;

            //Publisher
            rclcpp::Publisher <geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose;

            rclcpp::Publisher <nav_msgs::msg::Path>::SharedPtr pub_path;

            rclcpp::Publisher <geometry_msgs::msg::PoseStamped>::SharedPtr pub_ref_pose;

            //msg
            geometry_msgs::msg::PoseStamped current_pose;

            geometry_msgs::msg::PoseStamped ref_pose;

            nav_msgs::msg::Path path_taken;

            nav_msgs::msg::Odometry odom_pose_msg;

            nav_msgs::msg::Odometry odom_ref_pose_msg;

            //function 
            void process_cloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud_in, const rclcpp::Time stamp);

            void get_pose(const Eigen::Matrix4f final_transformation, const rclcpp::Time stamp);

            void ekf_scanmatching(Eigen::Vector3d translation_vector, const rclcpp::Time stamp, Eigen::Vector4d x);

            Eigen::Matrix4f getTransformation(const geometry_msgs::msg::Pose pose);

            //bool parameter
            bool initial_state_received_ {false};

            bool initial_cloud_received_ {false};

            bool ekf_initial_state_received_ {false};

            bool use_ekf_ {false};

            bool neglect_roll_pitch_z_ {false};

            //setting parameter

            double roll,pitch,yaw;

            double vg_size_;

            double R_variance_;

            //component
            Eigen::Vector4d x;

            Eigen::Matrix4d p;

            Eigen::Vector2d cumsum_vector;

            Eigen::Vector2d var_R;

            double cum_pose_x, cum_pose_y;

            long double previous_stamp_sensor;

            double previous_position_x, previous_position_y, previous_position_z;

            double pose_x, pose_y, pose_z;

            double previous_trans_pose_x, previous_trans_pose_y;

            Eigen::Matrix4d init_p_out_, p_predict_in, p_predict_out, p_correct_in, p_correct_out;

            Eigen::Vector4d init_x_out_, x_predict_in, x_predict_out, x_correct_in, x_correct_out;

            int i = 0;
            int u = 0;

            //ekf
            MY_SCANMATCHING_EKF ekf;
    };
}


#endif //AUTOBIN__SM_COMPONENT_HPP_




                