#ifndef AUTOBIN__MY_SLAM_COMPONENT_HPP_
#define AUTOBIN__MY_SLAM_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MY_SLAM_EXPORT __attribute__ ((dllexport))
    #define MY_SLAM_IMPORT __attribute__ ((dllimport))
  #else
    #define MY_SLAM_EXPORT __declspec(dllexport)
    #define MY_SLAM_IMPORT __declspec(dllimport)
  #endif
  #ifdef MY_SLAM_BUILDING_DLL
    #define MY_SLAM_PUBLIC MY_SLAM_EXPORT
  #else
    #define MY_SLAM_PUBLIC MY_SLAM_IMPORT
  #endif
  #define MY_SLAM_PUBLIC_TYPE MY_SLAM_PUBLIC
  #define MY_SLAM_LOCAL
#else
  #define MY_SLAM_EXPORT __attribute__ ((visibility("default")))
  #define MY_SLAM_IMPORT
  #if __GNUC__ >= 4
    #define MY_SLAM_PUBLIC __attribute__ ((visibility("default")))
    #define MY_SLAM_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MY_SLAM_PUBLIC
    #define MY_SLAM_LOCAL
  #endif
  #define MY_SLAM_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <my_package/ekf_chca.hpp>
#include <my_package/ekf_chcv.hpp>
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
#include <lidarslam_msgs/msg/map_array.hpp>
#include <kalman_msgs/msg/chca.hpp>
#include <kalman_msgs/msg/chcv.hpp>

#include <pclomp/ndt_omp.h>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp.h>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>
#include <pclomp/gicp_omp.h>
#include <pclomp/gicp_omp_impl.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>

#include <mutex>
#include <thread>
#include <future>
#include <iostream>

namespace autobin
{
    class SlamComponent : public rclcpp::Node
    {
        public:
            MY_SLAM_PUBLIC
            explicit SlamComponent(const rclcpp::NodeOptions & options);

        private:

            //set up tf2
            //===============================
            rclcpp::Clock my_clock;
            tf2_ros::Buffer my_buffer;
            tf2_ros::TransformListener my_listener;
            tf2_ros::TransformBroadcaster my_broadcaster;

            //set up frame id
            //===============================
            std::string global_frame_id;
            std::string robot_frame_id;

            //set up frame topic
            //===============================
            std::string cloud_topic;
            std::string odom_topic;

            //set up mapping parameter
            //===============================
            std::string registration_method;
            double trans_for_map_update;
            double map_publish_period;
            int num_targeted_cloud;

            //set up parameter for NDT
            //==============================
            double ndt_resolution;
            int ndt_num_threads;
            std::string neighborhood_search_method;

            //set up parameter for ICP
            //==============================
            double gicp_corr_dist_threshold;

            //set up parameter for Voxel Grid
            //==============================
            double vg_size, vg_size_map;
            double scan_min_range {1.0}, scan_max_range {100.0};

            //parameter for Kalman filtering
            //===============================
            double R_variance;
            std::string Kalman_Filter_model;
            int num_state_space {8};

            //parameter for Bool application
            //================================
            bool use_ekf {false};
            bool neglect_roll_pitch_z {false};

            //parameter for registration method
            //================================
            pcl::Registration <pcl::PointXYZI, pcl::PointXYZI>::Ptr registration;

            //odom_callback
            //================================
            int step {0};
            bool gps_available {true};
            bool initial_state_received {false};
            geometry_msgs::msg::PoseStamped current_pose;
            geometry_msgs::msg::PoseStamped ref_pose;
            geometry_msgs::msg::PoseStamped gps_pose;
            nav_msgs::msg::Path path_taken;
            nav_msgs::msg::Odometry odom_pose_msg;
            nav_msgs::msg::Odometry odom_ref_pose_msg;
            double previous_position_x, previous_position_y, previous_position_z;
            double pose_x, pose_y, pose_z;
            double diff_pose_x, diff_pose_y, diff_pose_z;

            //cloud_callback
            //================================
            geometry_msgs::msg::PoseStamped header_msg;
            tf2::TimePoint time_point;
            bool initial_cloud_received {false};
            lidarslam_msgs::msg::MapArray map_array_msg;
            rclcpp::Time last_map_time;

            //Subcriber
            //================================
            rclcpp::Subscription <nav_msgs::msg::Odometry>::SharedPtr sub_odom_pose;
            rclcpp::Subscription <sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_input;
            //rclcpp::Subscription <geometry_msgs::msg::PoseStamped>::SharedPtr sub_geo_pose;

            //Publisher
            //================================
            rclcpp::Publisher <geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose;
            rclcpp::Publisher <nav_msgs::msg::Path>::SharedPtr pub_path;
            rclcpp::Publisher <geometry_msgs::msg::PoseStamped>::SharedPtr pub_ref_pose;
            rclcpp::Publisher < sensor_msgs::msg::PointCloud2 > ::SharedPtr pub_map;
            rclcpp::Publisher < lidarslam_msgs::msg::MapArray > ::SharedPtr pub_map_array;
            rclcpp::Publisher < kalman_msgs::msg::Chca > ::SharedPtr pub_ekf_chca;
            rclcpp::Publisher < kalman_msgs::msg::Chcv > ::SharedPtr pub_ekf_chcv;

            //function process_cloud
            //================================
            void process_cloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud_in, const rclcpp::Time stamp);
            bool mapping_flag {false};
            std::future < void > mapping_future;
            bool is_map_updated {false};
            pcl::PointCloud < pcl::PointXYZI > targeted_cloud;
            std::thread mapping_thread;
            Eigen::Matrix<double, 8, 1> state_chca_out;
            Eigen::Matrix<double, 6, 1> state_chcv_out;

            //function get_pose
            //================================
            void get_pose(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud_in, const Eigen::Matrix4f final_transformation, const rclcpp::Time stamp);
            double trans;
            Eigen::Vector3d previous_position;
            Eigen::Vector3d translation_vector;
            Eigen::Matrix3d rotation_matrix;
            std::packaged_task < void() > mapping_task;

            //function get_pose
            //================================
            void update(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_in, const Eigen::Matrix4f final_transformation, const geometry_msgs::msg::PoseStamped current_pose_stamped);
            double latest_distance {0};

            //function publishMap
            //================================
            void publishMap();

            //function ekf_scanmatching model CHCA
            //================================
            kalman_msgs::msg::Chca kf_msg_chca;
            void ekf_scanmatching_CHCA(Eigen::Vector3d translation_vector,const double yaw, const rclcpp::Time stamp, Eigen::Matrix<double, 8, 1> &x_chca_out);
            Eigen::Matrix<double, 3, 1> measurement_vector;
            Eigen::Matrix<double, 3, 1> var_R;
            Eigen::Matrix<double, 8, 8> init_p_chca_out, p_predict_chca_in, p_predict_chca_out, p_correct_chca_in, p_correct_chca_out;
            Eigen::Matrix<double, 8, 1> init_x_chca_out, x_predict_chca_in, x_predict_chca_out, x_correct_chca_in, x_correct_chca_out;
            Eigen::Matrix<double, 8, 1> x_chca;
            Eigen::Matrix<double, 8, 8> p_chca;
            Eigen::Matrix<double, 8, 1> x_chca_out;
            bool ekf_initial_state_received {false};
            long double previous_stamp_sensor;
            EKF_CHCA ekf_chca;

            //function ekf_scanmatching model CHCA
            //================================
            kalman_msgs::msg::Chcv kf_msg_chcv;
            void ekf_scanmatching_CHCV(Eigen::Vector3d translation_vector,const double yaw, const rclcpp::Time stamp, Eigen::Matrix<double, 6, 1> &x_chcv_out);
            Eigen::Matrix<double, 6, 6> init_p_chcv_out, p_predict_chcv_in, p_predict_chcv_out, p_correct_chcv_in, p_correct_chcv_out;
            Eigen::Matrix<double, 6, 1> init_x_chcv_out, x_predict_chcv_in, x_predict_chcv_out, x_correct_chcv_in, x_correct_chcv_out;
            Eigen::Matrix<double, 6, 1> x_chcv;
            Eigen::Matrix<double, 6, 6> p_chcv;
            Eigen::Matrix<double, 6, 1> x_chcv_out;
            EKF_CHCV ekf_chcv;

            //function getTransformation
            //================================
            Eigen::Matrix4f getTransformation(const geometry_msgs::msg::Pose pose); 
    };
}

#endif //AUTOBIN__MY_SLAM_COMPONENT_HPP_