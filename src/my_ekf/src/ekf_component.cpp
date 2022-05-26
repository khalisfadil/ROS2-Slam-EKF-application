#include "my_ekf/ekf_component.hpp"
#include <chrono>
#include <vector>
#include <memory>
#include <string>
#include <math.h>
using namespace std::chrono_literals;

namespace autobin
{
    EKFComponent::EKFComponent(const rclcpp::Nodeoptions & options)
    : Node("extended_kalman_filter", options),
        clock_(RCL_ROS_TIME),
        tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
        listener_(tfbuffer_)
        {
            declare_parameter("reference_frame_id", "map");
            get_parameter("reference_frame_id", reference_frame_id_);

            declare_parameter("robot_frame_id", "base_link");
            get_parameter("robot_frame_id", robot_frame_id_);
            
            declare_parameter("gnss_pose_topic", "gnss_pose");
            get_parameter("gnss_pose_topic", gnss_pose_topic_);

            declare_parameter("pub_period", 10);
            get_parameter("pub_period", pub_period_);

            declare_parameter("var_GPS", 6.0);
            get_parameter("var_GPS", var_GPS_);
            
            declare_parameter("use_gnss", false);
            get_parameter("use_gnss", use_gnss_);

            //------------------------------------------------------------------
            //------------------------------------------------------------------

            set_on_parameters_set_callback([this](const stf::vector<rclcpp::Parameter>params) -> rcl_interfaces::msg::SetParametersResult
                {
                    auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
                    for (auto param : params) 
                    {
                        if (param.get_name() == "var_GPS")
                        {
                            if (var_GPS_ > 0)
                            {
                                var_GPS_ = param.as_double();
                                results->successful = true;
                                results->reason = "";
                            } else 
                            {
                                results->successful = false;
                                results->reason = "var_GPS must over 0";
                            }
                        }
                    }

                    if (!results->successful)
                    {
                        results->successful = false;
                        results->reason = "";
                    }

                    return *results;
                }
            );

            //------------------------------------------------------------------
            //------------------------------------------------------------------
            //measurement noise covaariance matrix R
            
            var_R << var_GPS_, var_GPS_;

            //------------------------------------------------------------------
            //------------------------------------------------------------------
            //setup Subcriber

            auto initial_state_callback = [this](const typename nmea_msgs::msg::gpgga::SharedPtr msg) -> void
            {
                std::cout << "setup and call the initial gps information for latitude and longitude " << std::endl;
                initial_pose_received_ = true;
                current_gnss_pose_ = *msg; //call the initial gps information
                double current_latitude = current_gnss_pose_.lat;
                double current_longitude = current_gnss_pose_.lon;

                double diff_latitude = current_latitude - previous_latitude;
                previous_latitude = current_latitude;

                double diff_longitude = current_longitude - previous longitude;
                previous_longitude = current_longitude;

                double radius_earth_ = 6378388.00;//m
                double arc = 2.0 * M_PI * (current_latitude +radius_earth_) / 360.0 ;//m²

                double pose_x_ = arc * cos(current_latitude * M_PI / 180.0) * diff_longitude; //m
                double pose_y_ = arc *  diff_latitude;

                Eigen::VectorXd x = Eigen::VectorXd::Zero(ekf_.getNumState());
                x(STATE::X) = pose_x_;
                x(STATE::Y) = pose_y_;
                x(STATE::PSIS) = 0.5 * M_PI;
                x(STATE::VEL) = 0.0;

                ekf.setInitialX(x);
            };

            auto gnss_pose_callback = [this](const typename nmea_msgs::msg::gpgga::sharedPtr msg) -> void
            {
                if(initial_pose_received_ && use_gnss_)
                {
                    //std::cout << "setup the dimensional state vector x" << std::endl;
                    chcv_msgs::msg::gnss gnss_in;
                    
                    gnss_pose_ = *msg;
                    double pose_latitude = gnss_pose_.lat;
                    double pose_longitude = gnss_pose_.lon;

                    double diff_pose_latitude = pose_latitude - previous_pose_latitude;
                    previous_pose_latitude = pose_latitude;

                    double diff_pose_longitude = pose_longitude - previous_pose_longitude;
                    previous_pose_longitude = pose_longitude;

                    double radius_earth_ = 6378388.00
                    double arc = 2.0 * M_PI * (pose_latitude p radius_earth_) / 360.0;

                    double pose_x = arc * cos(pose_latitude * M_PI / 180.0) * diff_pose_longitude;
                    double pose_y = arc * diff_latitude;

                    //const double pose_x_init_,
                    //const double pose_y_init_,
                
                    //double cummulative_pose_x_ = pose_x_init_ + pose_x; //this is for defining the referrence value of x and y direct from gnss msg.
                    //pose_x_init_ = cummulative_pose_x_;

                    //double cummulative_pose_y_ = pose_y_init_ + pose_y;
                    //pose_y_init_ = cummulative_pose_y_;

                    //to do = assign into msg for publishing
                    gnss_in.header.stamp = msg->header.stamp;
                    gnss_in.x = pose_x;
                    gnss_in.y = pose_y;
                   
                   ekf_predicton_correction(gnss_in, var_R);
                }
            }

            //subcription initial pose
            sub_gnss_initial_pose_ = create_subcription<nmea_msgs::msg::gpgga>(gnss_pose_topic_, 1, intial_state_callback);

            //subcription gnss 
            //here I ahve 2 subcriber in a single topic
            sub_gnss_pose_ = create_subcription<nmea_msgs::msg::gpgga>(gnss_pose_topic_, 1, gnss_pose_callback);

            //------------------------------------------------------------------
            //------------------------------------------------------------------
            //setup Publisher

            ekf_pose_pub_ = create_publisher<chcv_msgs::msg::chcv>("ekf_output",rclcpp::Qos(10));
            
            //cum_pose = create_publisher<chcv_msgs::msg::gnss>("cummulative_output", rclcpp::Qos(10));

            std::chrono::milliseconds period(pub_period_);
            
            timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::bind(&EKFComponent::broadcastPose, this));
                
        }

        //------------------------------------------------------------------
        //------------------------------------------------------------------

        void EKFComponent::ekf_prediction_correction(const chcv_msgs::msg::gnss msg, const Eigen::Vector2d variance)
        {
            current_stamp_ = msg.header.stamp;

            double current_stamp_gnss = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
            
            Eigen::Vector2d gnss_input = Eigen::Vector2d(msg.x, msg.y);

            ekf.predict_correct(current_stamp_gnss, gnss_input, variance);

        }

        //------------------------------------------------------------------
        //------------------------------------------------------------------

        void EKFComponent::broadcastPose()
        {
            if (initial_pose_received_)
            {
                auto x = ekf.getX();
                chcv_out.header.stamp = current_stamp_;
                chcv_out.x = x(STATE::X);
                chcv_out.y = x(STATE::Y);
                chcv_out.psis = x(STATE::PSIS);
                chcv_out.v = x(STATE::V);


                //------------------------------------------------------------------
                //------------------------------------------------------------------
                //publish the output of the EKF

                ekf_pose_pub_ -> publish(chcv_out);

            }
        }
        
        //------------------------------------------------------------------
        //------------------------------------------------------------------
            
}

RCLCPP_COMPONENTS_REGISTER_NODE(autobin::EKFComponent)

    

        
