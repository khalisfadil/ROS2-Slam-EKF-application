

#include <my_ekf/ekf_component.hpp>
#include <chrono>
#include <vector>
#include <memory>
#include <string>
#include <math.h>
using namespace std::chrono_literals;

namespace autobin
{
    EKFComponent::EKFComponent(const rclcpp::NodeOptions & options)
    : Node("extended_kalman_filter", options),
        clock_(RCL_ROS_TIME),
        tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
        listener_(tfbuffer_)
        {
            
            declare_parameter("gnss_pose_topic", "gnss_pose");
            get_parameter("gnss_pose_topic", gnss_pose_topic_);

            declare_parameter("var_GPS", 6.0);
            get_parameter("var_GPS", var_GPS_);

            declare_parameter("initialize_state_received", false);
            get_parameter("initialize_state_received", initialize_state_received_);

            
            //------------------------------------------------------------------
            //measurement noise covaariance matrix R
            //------------------------------------------------------------------
            var_R << pow(var_GPS_,2), pow(var_GPS_,2);

            //------------------------------------------------------------------
            //measurement noise covaariance matrix R
            //------------------------------------------------------------------

            auto ekf_callback = [this](const typename nmea_msgs::msg::Gpgga::SharedPtr msg) -> void
            {   
                chcv_msgs::msg::Chcv chcv_in;
                
                convert(msg, pose_x, pose_y);

                cumsum(pose_x, pose_y, cumsum_x, cumsum_y);

                if(!initialize_state_received_)
                {
                    initialize_state(pose_x, pose_y);

                    initialize_state_received_ = true;
                }else
                {
                    Eigen::Vector4d pre_state = Eigen::Vector4d::Zero(ekf.getNumState());
                    auto x = ekf.getX();
                    pre_state << cumsum_x, cumsum_y, x(STATE::PSIS), x(STATE::V);

                    chcv_in.header.stamp = msg->header.stamp;
                    chcv_in.x = pre_state(0);
                    chcv_in.y = pre_state(1);
                    chcv_in.psis = pre_state(2);
                    chcv_in.v = pre_state(3);

                    std::cout <<"============================================================================"<< std::endl;
                    std::cout <<"=================================ekf callback==============================="<< std::endl;
                    std::cout <<"x:  " << chcv_in.x << "    " <<"y:  " << chcv_in.y << "psis: " << chcv_in.psis << "     " << "v:   " << chcv_in.v << std::endl;
                    std::cout <<"============================================================================"<< std::endl;
                    
                    ekf_prediction(chcv_in, P);

                    ekf_correction(chcv_in, var_R, P);

                }

            };

            //------------------------------------------------------------------
            //setup Subcriber and Publisher
            //------------------------------------------------------------------

            sub_gnss_pose_ = create_subscription<nmea_msgs::msg::Gpgga>(gnss_pose_topic_, rclcpp::SensorDataQoS(), ekf_callback);

            ekf_pose_pub_ = create_publisher<chcv_msgs::msg::Chcv>("ekf_output",rclcpp::QoS(10));

            ekf_gps_pose_pub_ = create_publisher<chcv_msgs::msg::Gnss>("real_output", rclcpp::QoS(10));

            ekf_cov_pub_ = create_publisher<chcv_msgs::msg::Cov>("covariance_output", rclcpp::QoS(10));
                
        }

        //------------------------------------------------------------------
        //this function is to give the initial value to state x
        //------------------------------------------------------------------
        void EKFComponent::initialize_state(double pose_x_, double pose_y_)
        {
            ekf.initializing(pose_x_, pose_y_);
        }

        //------------------------------------------------------------------
        //this funciton is to convert the Gpgga msg into meter
        //------------------------------------------------------------------
        void EKFComponent::convert(const nmea_msgs::msg::Gpgga::SharedPtr msg, double &pose_x, double &pose_y)
        {
            gnss_pose_ = *msg;
            double pose_latitude = gnss_pose_.lat/100;
            double pose_longitude = gnss_pose_.lon/100;
            double pose_altitude = gnss_pose_.alt;

            double radius_earth_ = 6378388.0;
            double arc = 2.0 * M_PI * (pose_altitude + radius_earth_) / 360.0;

            pose_x = arc * cos(pose_latitude * M_PI / 180.0) * diff_pose_longitude;
            pose_y = arc * diff_pose_latitude;
            
            //make sure the first value is equal to zero
            if(previous_pose_latitude == 0 && previous_pose_longitude == 0)
            {
                previous_pose_latitude = pose_latitude;
                previous_pose_longitude = pose_latitude;
            }else{
                previous_pose_latitude = previous_pose_latitude;
                previous_pose_longitude = previous_pose_longitude;
            };

            diff_pose_latitude = -1*(pose_latitude - previous_pose_latitude);
            previous_pose_latitude = pose_latitude;

            diff_pose_longitude = -1*(pose_longitude - previous_pose_longitude);
            previous_pose_longitude = pose_longitude;

        }

        //------------------------------------------------------------------
        //this funciton is to calculate the cummulative sum of x and y
        //------------------------------------------------------------------
        void EKFComponent::cumsum(double pose_x_, double pose_y_, double &cum_pose_x_, double &cum_pose_y_)
        {
            cum_pose_x_ += pose_x_; 

            cum_pose_y_ += pose_y_;
        }

        //------------------------------------------------------------------
        //this funciton is to calculate the prediction from kalman filtering
        //------------------------------------------------------------------
        void EKFComponent::ekf_prediction(const chcv_msgs::msg::Chcv msg, Eigen::Matrix4d &P)
        {
            current_stamp_ = msg.header.stamp;

            double current_stamp_gnss = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
            
            Eigen::Vector4d state_value;

            state_value << msg.x, msg.y, msg.psis, msg.v;

            ekf.predict(current_stamp_gnss, P);
        }
        
        //------------------------------------------------------------------
        //this funciton is to calculate the correction from kalman filtering
        //------------------------------------------------------------------
        void EKFComponent::ekf_correction(const chcv_msgs::msg::Chcv msg, const Eigen::Vector2d variance, Eigen::Matrix4d P)
        {
            Eigen::Vector2d correction_state;

            correction_state << msg.x, msg.y;

            ekf.correct(correction_state, variance, P);
        }

        //------------------------------------------------------------------
        //------------------------------------------------------------------

        void EKFComponent::broadcastPose()
        {
 
            //get the latest value of state x
            auto x = ekf.getX();
            chcv_out.header.stamp = current_stamp_;
            chcv_out.x = x(STATE::X);
            chcv_out.y = x(STATE::Y);
            chcv_out.psis = x(STATE::PSIS);
            chcv_out.v = x(STATE::V);
            //publish the output of the EKF
            ekf_pose_pub_ -> publish(chcv_out);

            //-------------------------------------------------------------------
            //-------------------------------------------------------------------
            auto correction = ekf.getX_Correction();
            ref_out.header.stamp = current_stamp_;
            ref_out.x = correction(CORRECTIONSTATE::DX);
            ref_out.y = correction(CORRECTIONSTATE::DY);
            //publish the output of the real value / referrence value
            ekf_gps_pose_pub_ -> publish(ref_out);

            //-------------------------------------------------------------------
            //-------------------------------------------------------------------
            //get covariance output
            auto covariance = ekf.getX_Covariance();
            cov_out.header.stamp = current_stamp_;
            cov_out.p1 = covariance(COVARIANCESTATE::P1);
            cov_out.p2 = covariance(COVARIANCESTATE::P2);
            cov_out.p3 = covariance(COVARIANCESTATE::P3);
            cov_out.p4 = covariance(COVARIANCESTATE::P4);
            //publish covariance
            ekf_cov_pub_ -> publish(cov_out);

        }
        
        //------------------------------------------------------------------
        //------------------------------------------------------------------
            
}

RCLCPP_COMPONENTS_REGISTER_NODE(autobin::EKFComponent)

    

        
