#include <my_ekf/ekf.hpp>

#include <rclcpp/rclcpp.hpp>

#include <rclcpp_components/register_node_macro.hpp>

#include <eigen3/Core>

#include <string>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>

#include <chcv_msgs/msg/chcv.hpp>

#include <chcv_msgs/msg/gnss.hpp>

namespace autobin
{
    class EKFComponent : public rclcpp::Node;
    {
        public:
        //MY_EKF_PUBLIC
            explicit EKFComponent(const rclcpp::NodeOptions & options)

        private:
            std::string reference_frame_id_;
            std::string robot_frame_id_;
            std::string gnss_pose_topic_;
            int pub_period_;

            double var_GPS_;
            Eigen::Vector2d var_R;
            
            bool use_gnss_;

            bool initial_pose_received_{false};

            chcv_msgs::msg::chcv chcv_out;
            rclcpp::Time current_stamp_;

            EKF_CHCV ekf;

            //subcriber
            rclcpp::Subcription<nmea_msgs::msg::gpgga>::SharedPtr sub_gnss_initial_pose_;

            rclcpp::Subcription<nmea_msgs::msg::gpgga>::SharedPtr sub_gnss_pose_;

            //Publisher
            rclcpp::Publisher<chcv_msgs::msg::chcv>::SharedPtr ekf_pose_pub_;

            //Timer
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Clock clock_;
            
            //buffer and listener
            tf2_ros::Buffer tfbuffer_;
            tf2_ros::TransformListener listener_;

            //function
            

    }
}