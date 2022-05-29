
#ifndef AUTOBIN__EKF_COMPONENT_HPP_
#define AUTOBIN__EKF_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define A_EKFC_EXPORT __attribute__ ((dllexport))
    #define A_EKFC_IMPORT __attribute__ ((dllimport))
  #else
    #define A_EKFC_EXPORT __declspec(dllexport)
    #define A_EKFC_IMPORT __declspec(dllimport)
  #endif
  #ifdef A_EKFC_BUILDING_DLL
    #define A_EKFC_PUBLIC A_EKFC_EXPORT
  #else
    #define A_EKFC_PUBLIC A_EKFC_IMPORT
  #endif
  #define A_EKFC_PUBLIC_TYPE A_EKFC_PUBLIC
  #define A_EKFC_LOCAL
#else
  #define A_EKFC_EXPORT __attribute__ ((visibility("default")))
  #define A_EKFC_IMPORT
  #if __GNUC__ >= 4
    #define A_EKFC_PUBLIC __attribute__ ((visibility("default")))
    #define A_EKFC_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define A_EKFC_PUBLIC
    #define A_EKFC_LOCAL
  #endif
  #define A_EKFC_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif



#include <my_ekf/ekf.hpp>
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
#include <nmea_msgs/msg/gpgga.hpp>

#include <iostream>

namespace autobin
{
    class EKFComponent : public rclcpp::Node
    {
        public:
            A_EKFC_PUBLIC
            explicit EKFComponent(const rclcpp::NodeOptions & options);

        private:
            std::string reference_frame_id_;
            std::string robot_frame_id_;
            std::string gnss_pose_topic_;
            int pub_period_;

            double var_GPS_;
            Eigen::Vector2d var_R;
            
            bool use_gnss_;

            bool initial_pose_received_{false};

            chcv_msgs::msg::Chcv chcv_out;
            chcv_msgs::msg::Gnss gps_out;

            rclcpp::Time current_stamp_;

            EKF_CHCV ekf;

            nmea_msgs::msg::Gpgga current_gnss_pose_, gnss_pose_;
            double previous_latitude;
            double previous_longitude;
            double previous_pose_longitude;
            double previous_pose_latitude;
            double arc;
            double pose_x_init_;
            double pose_y_init_;
            chcv_msgs::msg::Gnss gnss_in;

            Eigen::Vector2d gps;


            //subcriber
            rclcpp::Subscription<nmea_msgs::msg::Gpgga>::SharedPtr sub_gnss_initial_pose_;

            rclcpp::Subscription<nmea_msgs::msg::Gpgga>::SharedPtr sub_gnss_pose_;

            //Publisher
            rclcpp::Publisher<chcv_msgs::msg::Chcv>::SharedPtr ekf_pose_pub_;

            rclcpp::Publisher<chcv_msgs::msg::Gnss>::SharedPtr ekf_gps_pose_pub_;


            //Timer
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Clock clock_;
            
            //buffer and listener
            tf2_ros::Buffer tfbuffer_;
            tf2_ros::TransformListener listener_;

            //function
            void ekf_prediction_correction(const chcv_msgs::msg::Gnss msg, const Eigen::Vector2d variance);
            void broadcastPose();

            enum STATE
            {
                X = 0, Y = 1, PSIS = 2, V =3,
            };

            enum GPSSTATE
            {
              DX = 0, DY = 1,
            };

    };
}

#endif  // AUTOBIN__EKF_COMPONENT_HPP_