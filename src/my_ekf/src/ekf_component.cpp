

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
            
            var_R << pow(var_GPS_,2), pow(var_GPS_,2);

            //------------------------------------------------------------------
            //------------------------------------------------------------------
            //setup Subcriber

            auto initial_state_callback = [this](const typename nmea_msgs::msg::Gpgga::SharedPtr msg) -> void
            {
                //RCLCPP_INFO(get_logger(), "initialization start");

                initial_pose_received_ = true;
                current_gnss_pose_ = *msg; //call the initial gps information
                double current_latitude = current_gnss_pose_.lat /100;
                double current_longitude = current_gnss_pose_.lon /100;
                double current_altitude = current_gnss_pose_.alt;

                double radius_earth_ = 6378388.0;//m
                double arc = 2.0 * M_PI * (current_altitude + radius_earth_) / 360.0 ;//m²

                double pose_x_ = arc * cos(current_latitude * M_PI / 180.0) * diff_longitude; //m
                double pose_y_ = arc *  diff_latitude;
                double pose_psis_ = 0.5*M_PI;

                if(previous_latitude == 0)
                {
                    previous_latitude = current_latitude;
                }else{
                    previous_latitude = previous_latitude;
                };

                if(previous_longitude == 0)
                {
                    previous_longitude = current_longitude;
                }else{
                    previous_longitude = previous_longitude;
                };

                diff_latitude = current_latitude - previous_latitude; //let say the array [1-0] [1.1-1][1.2-1.1]
                previous_latitude = current_latitude;

                diff_longitude = current_longitude - previous_longitude;
                previous_longitude = current_longitude;

                Eigen::Vector4d x = Eigen::Vector4d::Zero(ekf.getNumState());
               //Eigen::Map<Eigen::VectorXd> x;
                x(STATE::X) = pose_x_; // = 0
                x(STATE::Y) = pose_y_; // = 0
                x(STATE::PSIS) = pose_psis_; // 1/2* M_PI;
                x(STATE::V) = 0.01;

                ekf.setInitialState(x);
                //std::cout << "pose PSIS:  " << std::endl;

                //RCLCPP_INFO(get_logger(), "initialization end");

                //initialized_finished = true;

                //std::cout <<"============================================================================"<< std::endl;
                //std::cout <<"====================== initial state-callback==============================="<< std::endl;
                //std::cout <<" Lat:  " << pose_latitude << "    " <<"Lon:  " << pose_longitude << std::endl;
                //std::cout <<" X:  " <<  x(STATE::X) << "    " << "Y:  " << x(STATE::Y)<< "    " << "Yaw:  " << x(STATE::PSIS) << "    " << "Vel:  " << x(STATE::V) << std::endl;
                //std::cout <<"============================================================================"<< std::endl;
            };

            auto gnss_pose_callback = [this](const typename nmea_msgs::msg::Gpgga::SharedPtr msg) -> void
            {
                if(initial_pose_received_)
                {
                    //std::cout << "setup the dimensional state vector x" << std::endl;
                    chcv_msgs::msg::Gnss gnss_in;
                    
                    gnss_pose_ = *msg;
                    double pose_latitude = gnss_pose_.lat/100;
                    double pose_longitude = gnss_pose_.lon/100;
                    double pose_altitude = gnss_pose_.alt;

                    double radius_earth_ = 6378388.0;
                    double arc = 2.0 * M_PI * (pose_altitude + radius_earth_) / 360.0;

                    double pose_x = arc * cos(pose_latitude * M_PI / 180.0) * diff_pose_longitude;
                    double pose_y = arc * diff_pose_latitude;

                    if(previous_pose_latitude == 0)
                    {
                        previous_pose_latitude = pose_latitude;
                    }else{
                        previous_pose_latitude = previous_pose_latitude;
                    };

                    if(previous_pose_longitude == 0)
                    {
                        previous_pose_longitude = pose_longitude;
                    }else{
                        previous_pose_longitude = previous_pose_longitude;
                    };

                    diff_pose_latitude = pose_latitude - previous_pose_latitude;
                    previous_pose_latitude = pose_latitude;

                    diff_pose_longitude = pose_longitude - previous_pose_longitude;
                    previous_pose_longitude = pose_longitude;
                
                    double cummulative_pose_x_ = pose_x_next_ + pose_x; //this is for defining the referrence value of x and y direct from gnss msg.
                    pose_x_next_ = cummulative_pose_x_;

                    double cummulative_pose_y_ = pose_y_next_ + pose_y;
                    pose_y_next_ = cummulative_pose_y_;

                    //to do = assign into msg for publishing
                    gnss_in.header.stamp = msg->header.stamp;
                    gnss_in.x = cummulative_pose_x_;
                    gnss_in.y = cummulative_pose_y_;

                    std::cout <<"============================================================================"<< std::endl;
                    std::cout <<"=============================GNSS-in callback==============================="<< std::endl;
                    std::cout <<" Lat:  " << pose_latitude << "    " <<"Lon:  " << pose_longitude << "    " <<"Alt:  " << pose_altitude <<std::endl;
                    std::cout <<" gnss.x:  " << gnss_in.x << "    " <<"gnss.y:  " << gnss_in.y << std::endl;
                    std::cout << "X: " << pose_x << "     " << "Y:   " << pose_y << std::endl;
                    std::cout << "diff lat: " << diff_pose_latitude << "     " << "diff lat: " << diff_pose_longitude << std::endl;
                    std::cout <<"============================================================================"<< std::endl;
                    
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

            ekf_gps_pose_pub_ = create_publisher<chcv_msgs::msg::Gnss>("real_output", rclcpp::QoS(10));

            ekf_cov_pub_ = create_publisher<chcv_msgs::msg::Cov>("covariance_output", rclcpp::QoS(10));
            
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
                auto gps = ekf.getX_GPS();
                gps_out.header.stamp = current_stamp_;
                gps_out.x = gps(GPSSTATE::DX);
                gps_out.y = gps(GPSSTATE::DY);
                //publish the output of the real value / referrence value
                ekf_gps_pose_pub_ -> publish(gps_out);
                //-------------------------------------------------------------------
                //-------------------------------------------------------------------
                //get covariance output
                auto covariance = ekf.getMatrixCovariance();
                cov_out.header.stamp = current_stamp_;
                cov_out.p1 = covariance(COVARIANCESTATE::P1);
                cov_out.p2 = covariance(COVARIANCESTATE::P2);
                cov_out.p3 = covariance(COVARIANCESTATE::P3);
                cov_out.p4 = covariance(COVARIANCESTATE::P4);
                //publish covariance
                ekf_cov_pub_ -> publish(cov_out);
            }
        }
        
        //------------------------------------------------------------------
        //------------------------------------------------------------------
            
}

RCLCPP_COMPONENTS_REGISTER_NODE(autobin::EKFComponent)

    

        
