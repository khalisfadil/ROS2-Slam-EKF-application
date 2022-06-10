#ifndef AUTOBIN__EKF_KITTI_COMPONENT_HPP_
#define AUTOBIN__EKF_KITTI_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define A_EKFKC_EXPORT __attribute__ ((dllexport))
    #define A_EKFKC_IMPORT __attribute__ ((dllimport))
  #else
    #define A_EKFKC_EXPORT __declspec(dllexport)
    #define A_EKFKC_IMPORT __declspec(dllimport)
  #endif
  #ifdef A_EKFKC_BUILDING_DLL
    #define A_EKFKC_PUBLIC A_EKFKC_EXPORT
  #else
    #define A_EKFKC_PUBLIC A_EKFKC_IMPORT
  #endif
  #define A_EKFKC_PUBLIC_TYPE A_EKFKC_PUBLIC
  #define A_EKFKC_LOCAL
#else
  #define A_EKFKC_EXPORT __attribute__ ((visibility("default")))
  #define A_EKFKC_IMPORT
  #if __GNUC__ >= 4
    #define A_EKFKC_PUBLIC __attribute__ ((visibility("default")))
    #define A_EKFKC_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define A_EKFKC_PUBLIC
    #define A_EKFKC_LOCAL
  #endif
  #define A_EKFKC_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif



#include <my_ekf_kitti/ekf_kitti.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rclcpp_components/register_node_macro.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <string>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>

#include <chcv_msgs/msg/chcv.hpp>
#include <chcv_msgs/msg/gnss.hpp>
#include <chcv_msgs/msg/cov.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <iostream>

namespace autobin
{
    class EKFKITTIComponent : public rclcpp::Node
    {
        public:
            A_EKFKC_PUBLIC
            explicit EKFKITTIComponent(const rclcpp::NodeOptions & options);

        private:

            std::string gnss_pose_topic_;

            double var_GPS_;
            Eigen::Vector2d var_R;
            
            //auto ekf_callback
            bool initialize_state_received_;
            chcv_msgs::msg::Chcv chcv_in;
            Eigen::Matrix4d init_p_out_, p_predict_in, p_predict_out, p_correct_in, p_correct_out;
            Eigen::Vector4d init_x_out_, x_predict_in, x_predict_out, x_correct_in, x_correct_out;
            int i = 0;



            //void EKFKITTIComponent::broadcastPose()
            chcv_msgs::msg::Chcv chcv_predict_out;
            chcv_msgs::msg::Chcv chcv_correct_out;
            chcv_msgs::msg::Cov cov_out;
            chcv_msgs::msg::Gnss gps_pose_out;

            rclcpp::Time current_stamp_;

            //class EKFKITTI_CHCV from ekf_kitti.hpp
            EKFKITTI_CHCV ekf;

            //topic name of gnss pose subs
            sensor_msgs::msg::NavSatFix gnss_pose_;

            //void EKFKITTIComponent::convert
            double diff_pose_latitude;
            double diff_pose_longitude;
            double previous_pose_longitude;
            double previous_pose_latitude;
            double arc;
            double pose_x, pose_y;

            //void EKFKITTIComponent::cumsum
            double cumsum_x, cumsum_y;

            //void EKFKITTIComponent::ekf_prediction
            long double previous_stamp_gnss;

            //EKFKITTICOMPONENT::ekf_callback
            Eigen::Vector4d x;
            Eigen::Matrix4d p;
            Eigen::Vector2d cumsum_vec;
            
            //subcriber

            rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gnss_pose_;

            //Publisher
            rclcpp::Publisher<chcv_msgs::msg::Chcv>::SharedPtr ekf_predict_pub_;

            rclcpp::Publisher<chcv_msgs::msg::Chcv>::SharedPtr ekf_correct_pub_;

            rclcpp::Publisher<chcv_msgs::msg::Gnss>::SharedPtr ekf_gps_pose_pub_;

            rclcpp::Publisher<chcv_msgs::msg::Cov>::SharedPtr  ekf_cov_pub_;

            //Timer
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Clock clock_;
            
            //buffer and listener
            tf2_ros::Buffer tfbuffer_;
            tf2_ros::TransformListener listener_;

            //function //TODO UPDATE FUNCTION
            void initialize_state(double pose_x_, double pose_y_, Eigen::Vector4d &X_init_out, Eigen::Matrix4d &P_init_out);
            void convert(const sensor_msgs::msg::NavSatFix::SharedPtr msg, double &pose_x, double &pose_y);
            void cumsum(double pose_x_, double pose_y_, double &cum_pose_x_, double &cum_pose_y_);
            void ekf_prediction(const chcv_msgs::msg::Chcv msg, Eigen::Vector4d x_in, Eigen::Matrix4d p_in, Eigen::Vector4d &x_out, Eigen::Matrix4d &p_out);
            void ekf_correction(const Eigen::Vector2d variance, Eigen::Vector2d correction_state, Eigen::Vector4d x_cor_in, Eigen::Matrix4d p_cor_in, Eigen::Vector4d &x_cor_out, Eigen::Matrix4d &p_cor_out);
            void broadcastPose();

            enum STATE
            {
                X =  0 , Y = 1 , PSIS = 2 , V = 3
            };


            enum PREDICTIONSTATE
            {
                X_ =  0 , Y_ = 1 , PSIS_ = 2 , V_ = 3
            };

            enum CORRECTIONSTATE
            {
                DX = 0 , DY = 1, DPSIS = 2, DV = 3
            };

            enum COVARIANCESTATE
            {
                P1 = 0, P2 = 1, P3 = 2, P4 = 3
            };

    };
}

#endif  // AUTOBIN__EKF_KITTI_COMPONENT_HPP_