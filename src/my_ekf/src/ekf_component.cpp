

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
            //declare_parameter("reference_frame_id", "map");
            //get_parameter("reference_frame_id", reference_frame_id_);

            //declare_parameter("robot_frame_id", "base_link");
            //get_parameter("robot_frame_id", robot_frame_id_);
            
            declare_parameter("gnss_pose_topic", "gnss_pose");
            get_parameter("gnss_pose_topic", gnss_pose_topic_);

            declare_parameter("pub_period", 3);
            get_parameter("pub_period", pub_period_);

            declare_parameter("var_GPS", 6.0);
            get_parameter("var_GPS", var_GPS_);
            
            declare_parameter("use_gnss", true);
            get_parameter("use_gnss", use_gnss_);

            std::cout << "gnss_pose_topic:" << gnss_pose_topic_ << std::endl;

            //------------------------------------------------------------------
            //------------------------------------------------------------------
            
            set_on_parameters_set_callback([this](const std::vector<rclcpp::Parameter>params) -> rcl_interfaces::msg::SetParametersResult
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

            auto initial_state_callback = [this](const typename nmea_msgs::msg::Gpgga::SharedPtr msg) -> void
            {
                RCLCPP_INFO(get_logger(), "initialization start");

                initial_pose_received_ = true;
                current_gnss_pose_ = *msg; //call the initial gps information
                double current_latitude = current_gnss_pose_.lat;
                double current_longitude = current_gnss_pose_.lon;

                std::cout << "current latitude:  " << current_latitude << "    " <<"current longitude:  " << current_longitude << std::endl;

                double diff_latitude = current_latitude - previous_latitude;
                previous_latitude = current_latitude;

                double diff_longitude = current_longitude - previous_longitude;
                previous_longitude = current_longitude;
                
                
                std::cout << "diff latitude:  " << diff_latitude  << "    " << "diff_longitude:  " << diff_longitude << std::endl;

                double radius_earth_ = 6378388.00;//m
                double arc = 2.0 * M_PI * (current_latitude +radius_earth_) / 360.0 ;//m²

                std::cout << "arc:  " << arc  << std::endl;

                float pose_x_ = arc * cos(current_latitude * M_PI / 180.0) * diff_longitude; //m
                float pose_y_ = arc *  diff_latitude;
                float pose_psis_ = 0.5*M_PI;

                std::cout << "pose X:  " << pose_x_ << "    " << "pose Y:  " << pose_y_<< "    " << "pose psis:  " << pose_psis_ << std::endl;

                Eigen::Vector4d x = Eigen::Vector4d::Zero(ekf.getNumState());
               //Eigen::Map<Eigen::VectorXd> x;
                x(STATE::X) = pose_x_;
                x(STATE::Y) = pose_y_;
                x(STATE::PSIS) = pose_psis_; //* M_PI;
                x(STATE::V) = 0.0;

                ekf.setInitialState(x);
                
                //std::cout << "pose PSIS:  " << std::endl;

                RCLCPP_INFO(get_logger(), "initialization end");

                initialized_finished = true;
            };

            auto gnss_pose_callback = [this](const typename nmea_msgs::msg::Gpgga::SharedPtr msg) -> void
            {
                if(initial_pose_received_ && initialized_finished)
                {
                    //std::cout << "setup the dimensional state vector x" << std::endl;
                    chcv_msgs::msg::Gnss gnss_in;
                    
                    gnss_pose_ = *msg;
                    double pose_latitude = gnss_pose_.lat;
                    double pose_longitude = gnss_pose_.lon;

                    double diff_pose_latitude = pose_latitude - previous_pose_latitude;
                    previous_pose_latitude = pose_latitude;

                    double diff_pose_longitude = pose_longitude - previous_pose_longitude;
                    previous_pose_longitude = pose_longitude;

                    double radius_earth_ = 6378388.00;
                    double arc = 2.0 * M_PI * (pose_latitude + radius_earth_) / 360.0;

                    double pose_x = arc * cos(pose_latitude * M_PI / 180.0) * diff_pose_longitude;
                    double pose_y = arc * diff_pose_latitude;

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

                    std::cout << "x:  " << pose_x << "    " << "y:  " << pose_y << std::endl;
                   
                   ekf_prediction_correction(gnss_in, var_R);
                }
            };

            //subcription initial pose
            sub_gnss_initial_pose_ = create_subscription<nmea_msgs::msg::Gpgga>(gnss_pose_topic_, rclcpp::SensorDataQoS(), initial_state_callback);

            //subcription gnss 
            //here I ahve 2 subcriber in a single topic
            sub_gnss_pose_ = create_subscription<nmea_msgs::msg::Gpgga>(gnss_pose_topic_, rclcpp::SensorDataQoS(), gnss_pose_callback);

            //------------------------------------------------------------------
            //------------------------------------------------------------------
            //setup Publisher

            ekf_pose_pub_ = create_publisher<chcv_msgs::msg::Chcv>("ekf_output",rclcpp::QoS(10));
            
            //cum_pose = create_publisher<chcv_msgs::msg::gnss>("cummulative_output", rclcpp::Qos(10));

            std::chrono::milliseconds period(pub_period_);
            
            timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::bind(&EKFComponent::broadcastPose, this));
                
        }

        //------------------------------------------------------------------
        //------------------------------------------------------------------

        void EKFComponent::ekf_prediction_correction(const chcv_msgs::msg::Gnss msg, const Eigen::Vector2d variance)
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

    

        
