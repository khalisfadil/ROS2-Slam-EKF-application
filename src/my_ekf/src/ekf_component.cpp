

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

                cumsum_vec << cumsum_x, cumsum_y;

                if(!initialize_state_received_)
                {

                    initialize_state(pose_x, pose_y, init_x_out_, init_p_out_); //declare x and p globally
                    
                    x = init_x_out_;

                    p = init_p_out_;

                    chcv_in.x = x(0);
                    chcv_in.y = x(1);
                    chcv_in.psis = x(2);
                    chcv_in.v = x(3);

                    initialize_state_received_ = true;

                    broadcastPose();

                }else if(initialize_state_received_)
                {
                    //x << cumsum_x, cumsum_y, x(2), x(3);

                    chcv_in.header.stamp = msg->header.stamp; 
                    //x vector is declared globally
                    x_predict_in = x;

                    p_predict_in = p;

                    ekf_prediction(chcv_in, x_predict_in, p_predict_in, x_predict_out, p_predict_out);

                    p_correct_in = p_predict_out;

                    x_correct_in = x_predict_out; //TODO function x_correct_in = x if correction is not used

                    ekf_correction(var_R, cumsum_vec, x_correct_in, p_correct_in, x_correct_out, p_correct_out);

                    p = p_correct_out;

                    x = x_correct_out;

                    broadcastPose();

                    //checking the matrix

                    i++; 

                    std::cout <<"============================================================================"<< std::endl;
                    std::cout <<"=================================ekf callback==============================="<< std::endl;
                    std::cout <<"STEP:  "<< i << std::endl;
                    std::cout <<"                                                                            " <<std::endl;
                    std::cout <<"X_in:  " << x_predict_in(0) << "    " <<"Y_in:  " << x_predict_in(1) << "    " << "Yaw_in: " << x_predict_in(2) << "     " << "V_in:   " << x_predict_in(3) << std::endl; 
                    std::cout <<"X_out:  " << x(0) << "    " <<"Y_out:  " << x(1) << "    " << "Yaw_out: " << x(2) << "     " << "V_out:   " << x(3) << std::endl;                  
                    std::cout <<"P1: " <<  p(0,0) << "    " << "P2:  " << p(1,1)<< "    " << "P3:  " << p(2,2) << "    " << "P4:  " << p(3,3) << std::endl;
                    std::cout <<"============================================================================"<< std::endl;

                }

            };

            //------------------------------------------------------------------
            //setup Subcriber and Publisher
            //------------------------------------------------------------------

            sub_gnss_pose_ = create_subscription<nmea_msgs::msg::Gpgga>(gnss_pose_topic_, rclcpp::SensorDataQoS(), ekf_callback);

            ekf_predict_pub_ = create_publisher<chcv_msgs::msg::Chcv>("ekf_predict_output",rclcpp::QoS(10));

            ekf_correct_pub_ = create_publisher<chcv_msgs::msg::Chcv>("ekf_correct_output",rclcpp::QoS(10));

            ekf_gps_pose_pub_ = create_publisher<chcv_msgs::msg::Gnss>("real_output", rclcpp::QoS(10));

            ekf_cov_pub_ = create_publisher<chcv_msgs::msg::Cov>("covariance_output", rclcpp::QoS(10));
                
        }

        //------------------------------------------------------------------
        //this function is to give the initial value to state x
        //------------------------------------------------------------------
        void EKFComponent::initialize_state(double pose_x_, double pose_y_, Eigen::Vector4d &X_init_out, Eigen::Matrix4d &P_init_out)
        {
            ekf.initializing(pose_x_, pose_y_, X_init_out, P_init_out);
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

            //make sure the first value is equal to zero
            if(previous_pose_latitude == 0 && previous_pose_longitude == 0)
            {
                previous_pose_latitude = pose_latitude;
                previous_pose_longitude = pose_longitude;
            }else{
                previous_pose_latitude = previous_pose_latitude;
                previous_pose_longitude = previous_pose_longitude;
            };

            diff_pose_latitude = (pose_latitude - previous_pose_latitude);
            previous_pose_latitude = pose_latitude;

            diff_pose_longitude = (pose_longitude - previous_pose_longitude);
            previous_pose_longitude = pose_longitude;

            pose_x = arc * cos(pose_latitude * M_PI / 180.0) * diff_pose_longitude;
            pose_y = arc * diff_pose_latitude;

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
        void EKFComponent::ekf_prediction(const chcv_msgs::msg::Chcv msg, Eigen::Vector4d x_in, Eigen::Matrix4d p_in, Eigen::Vector4d &x_out, Eigen::Matrix4d &p_out)
        {
            current_stamp_ = msg.header.stamp;

            long double current_stamp_gnss = msg.header.stamp.sec + (msg.header.stamp.nanosec *1e-9);

            if(previous_stamp_gnss == 0)
            {
                previous_stamp_gnss = current_stamp_gnss;
            }

            long double dt = current_stamp_gnss - previous_stamp_gnss;
            previous_stamp_gnss = current_stamp_gnss;

            ekf.predict(dt, x_in, p_in, x_out, p_out);
        }
        
        //------------------------------------------------------------------
        //this funciton is to calculate the correction from kalman filtering
        //------------------------------------------------------------------
        void EKFComponent::ekf_correction(const Eigen::Vector2d variance, Eigen::Vector2d correction_state, Eigen::Vector4d x_cor_in, Eigen::Matrix4d p_cor_in, Eigen::Vector4d &x_cor_out, Eigen::Matrix4d &p_cor_out)
        {
            ekf.correct(variance, correction_state, x_cor_in, p_cor_in, x_cor_out, p_cor_out);
        }

        //------------------------------------------------------------------
        //------------------------------------------------------------------

        void EKFComponent::broadcastPose()
        {
 
            //get the latest value of prediction state
            auto prediction_state = ekf.getX_Prediction();
            chcv_predict_out.header.stamp = current_stamp_;
            chcv_predict_out.x = prediction_state(PREDICTIONSTATE::X_);
            chcv_predict_out.y = prediction_state(PREDICTIONSTATE::Y_);
            chcv_predict_out.psis = prediction_state(PREDICTIONSTATE::PSIS_);
            chcv_predict_out.v = prediction_state(PREDICTIONSTATE::V_);
            //publish the output of the EKF predict
            ekf_predict_pub_ -> publish(chcv_predict_out);

            //-------------------------------------------------------------------
            //-------------------------------------------------------------------
            //get the latest value of correction state
            auto correction_state = ekf.getX_Correction();
            chcv_correct_out.header.stamp = current_stamp_;
            chcv_correct_out.x = correction_state(CORRECTIONSTATE::DX);
            chcv_correct_out.y = correction_state(CORRECTIONSTATE::DY);
            chcv_correct_out.psis = correction_state(CORRECTIONSTATE::DPSIS);
            chcv_correct_out.v = correction_state(CORRECTIONSTATE::DV);
            //publish the output of the EKF correct
            ekf_correct_pub_ -> publish(chcv_correct_out);
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

            //TODO GNSS POSE PUB
            gps_pose_out.header.stamp = current_stamp_;
            gps_pose_out.x = cumsum_x;
            gps_pose_out.y = cumsum_y;
            //publish gps pose
            ekf_gps_pose_pub_ -> publish(gps_pose_out);

        }
        
        //------------------------------------------------------------------
        //------------------------------------------------------------------
            
}

RCLCPP_COMPONENTS_REGISTER_NODE(autobin::EKFComponent)

    

        
