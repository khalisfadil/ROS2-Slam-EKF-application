#include <my_scanmatching_model_chcv/scanmatching_model_chcv_component.hpp>
#include <chrono>

using namespace std::chrono_literals;

namespace autobin
{
    ScanmatchingComponent::ScanmatchingComponent(const rclcpp::NodeOptions & options)
    : Node("scan_matching", options),
    clock_(RCL_ROS_TIME),
    tfbuffer(std::make_shared<rclcpp::Clock>(clock_)),
    listener(tfbuffer),
    broadcaster(this)
    {   
        RCLCPP_INFO(get_logger(), "initialization CHCV start");

        declare_parameter("global_frame_id_", "map");
        get_parameter("global_frame_id_", global_frame_id_);

        declare_parameter("robot_frame_id", "base_link");
        get_parameter("robot_frame_id", robot_frame_id_);

        declare_parameter("lidar_frame_id", "front_lidar");
        get_parameter("lidar_frame_id", lidar_frame_id_);
        
        declare_parameter("registration_method", "NDT");
        get_parameter("registration_method", registration_method_);

        declare_parameter("cloud_topic", "cloud_topic");
        get_parameter("cloud_topic", cloud_topic_);

        declare_parameter("odom_topic", "odom_topic");
        get_parameter("odom_topic", odom_topic_);

        //=====================================================
        //test for NDT
        //=====================================================
        declare_parameter("trans_for_mapupdate", 1.5);
        get_parameter("trans_for_mapupate", trans_for_mapupdate);

        declare_parameter("scan_period", 0.1);
        get_parameter("scan_period", scan_period_);

        declare_parameter("map_publish_period", 15.0);
        get_parameter("mpa_publish_period", map_publish_period);

        declare_parameter("num_targeted_cloud", 10);
        get_parameter("num_targeted_cloud", num_targeted_cloud);

        if (num_targeted_cloud < 1) {
        std::cout << "num_tareged_cloud should be positive" << std::endl;
        num_targeted_cloud = 1;
  }

        //=====================================================
        //parameter for NDT
        //=====================================================

        declare_parameter("ndt_resolution", 1.0);
        get_parameter("ndt_resolution", ndt_resolution_);

        declare_parameter("ndt_num_threads", 8);
        get_parameter("ndt_num_threads", ndt_num_threads_);

        //=====================================================
        //parameter for ICP
        //=====================================================

        declare_parameter("gicp_corr_dist_threshold", 5.0);
        get_parameter("gicp_corr_dist_threshold", gicp_corr_dist_threshold_);

        //=====================================================
        //parameter for Vogel Grid
        //=====================================================

        declare_parameter("vg_size", 0.1);
        get_parameter("vg_size", vg_size_);

        declare_parameter("vg_size_map", 0.1);
        get_parameter("vg_size_map", vg_size_map);

        //=====================================================
        //parameter for ExtendedKalmanFilter
        //=====================================================

        declare_parameter("R_variance", 6.0);
        get_parameter("R_variance", R_variance_);

        //=====================================================
        //parameter for bool application
        //=====================================================

        declare_parameter("use_ekf", false);
        get_parameter("use_ekf", use_ekf_);

        declare_parameter("neglect_roll_pitch_z", false);
        get_parameter("neglect_roll_pitch_z", neglect_roll_pitch_z_);

        //=====================================================
        //parameter statement
        //=====================================================
        std::cout << "------------------" << std::endl;
        std::cout << "registration_method:" << registration_method_ << std::endl;
        std::cout << "voxel_leaf_size[m]:" << vg_size_ << std::endl;
        std::cout << "ndt_resolution[m]:" << ndt_resolution_ << std::endl;
        std::cout << "ndt_num_threads:" << ndt_num_threads_ << std::endl;
        std::cout << "use_ekf:" << std::boolalpha << use_ekf_ << std::endl;
        std::cout << "neglect_roll_pitch_z:" << std::boolalpha << neglect_roll_pitch_z_ << std::endl;
        std::cout << "------------------" << std::endl;

        path_taken.header.frame_id=global_frame_id_;

        map_array_msg_.header.frame_id = global_frame_id_;
        map_array_msg_.cloud_coordinate = map_array_msg_.LOCAL;



        //=====================================================
        //describe the registration
        //=====================================================
        if(registration_method_ == "NDT_OMP")
        {
            pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
            
            ndt_omp->setResolution(ndt_resolution_);

            ndt_omp->setTransformationEpsilon(0.01);

            ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT1);

            if (ndt_num_threads_ >0)
            {
                ndt_omp->setNumThreads(ndt_num_threads_);
            }

            registration = ndt_omp;

        }if(registration_method_ == "GICP_OMP")
        {
            pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr gicp_omp(new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());

            gicp_omp->setMaxCorrespondenceDistance(gicp_corr_dist_threshold_);

            gicp_omp->setTransformationEpsilon(0.01);

            registration = gicp_omp;

        }

        //=====================================================
        //run scanmatching process
        //=====================================================


        auto odom_callback = [this](const typename nav_msgs::msg::Odometry::SharedPtr msg) ->void
        {
            //=====================================================
            //ilustrate the gps available and not avalable condition
            //=====================================================
            z++;
            std::cout<< "step: " << z << std::endl;
            if(z <= 10 || z>= 70)
            {
                gps_available = true;
            }else
            {
                gps_available = false;
            }

            if(!initial_state_received_)
            {
                odom_pose_msg = *msg;

                auto geo_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
                geo_msg->header.frame_id= global_frame_id_;
                geo_msg->header.stamp = header_msg.header.stamp;
                geo_msg->pose.position.x = 0;
                geo_msg->pose.position.y = 0;
                geo_msg->pose.position.z = 0;
                geo_msg->pose.orientation.x = odom_pose_msg.pose.pose.orientation.x;
                geo_msg->pose.orientation.y = odom_pose_msg.pose.pose.orientation.y;
                geo_msg->pose.orientation.z = odom_pose_msg.pose.pose.orientation.z;
                geo_msg->pose.orientation.w = odom_pose_msg.pose.pose.orientation.w;
                current_pose = *geo_msg;
                pub_pose -> publish(current_pose);

                path_taken.poses.push_back(current_pose);

                initial_state_received_ = true;
            }

            //get the value direct from nav msg to be as referrence value
            odom_ref_pose_msg = *msg;

            if(previous_position_x == 0 && previous_position_y == 0 && previous_position_z == 0)
            {
                previous_position_x = odom_ref_pose_msg.pose.pose.position.x;
                previous_position_y = odom_ref_pose_msg.pose.pose.position.y;
                previous_position_z = odom_ref_pose_msg.pose.pose.position.z;
            }

            ref_pose.header = header_msg.header;
            ref_pose.header.frame_id = global_frame_id_;
            ref_pose.header.stamp = header_msg.header.stamp;

            double diff_pose_x = odom_ref_pose_msg.pose.pose.position.x - previous_position_x;
            double diff_pose_y = odom_ref_pose_msg.pose.pose.position.y - previous_position_y;
            double diff_pose_z = odom_ref_pose_msg.pose.pose.position.z - previous_position_z;

            pose_x += diff_pose_x;
            pose_y += diff_pose_y;
            pose_z += diff_pose_z;

            ref_pose.pose.position.x = pose_x;
            ref_pose.pose.position.y = pose_y;
            ref_pose.pose.position.z = pose_z;
            ref_pose.pose.orientation.x = odom_pose_msg.pose.pose.orientation.x;
            ref_pose.pose.orientation.y = odom_pose_msg.pose.pose.orientation.y;
            ref_pose.pose.orientation.z = odom_pose_msg.pose.pose.orientation.z;
            ref_pose.pose.orientation.w = odom_pose_msg.pose.pose.orientation.w;

            pub_ref_pose -> publish(ref_pose);

            previous_position_x = odom_ref_pose_msg.pose.pose.position.x;
            previous_position_y = odom_ref_pose_msg.pose.pose.position.y;
            previous_position_z = odom_ref_pose_msg.pose.pose.position.z;

            if (gps_available)
            {
                RCLCPP_INFO(get_logger(), "GPS available - take pose from GPS");

                gps_pose.header.frame_id = global_frame_id_;
                gps_pose.header.stamp = header_msg.header.stamp;
                gps_pose.pose.position.x = pose_x;
                gps_pose.pose.position.y = pose_y;
                gps_pose.pose.position.z = pose_z;
                gps_pose.pose.orientation.x = odom_pose_msg.pose.pose.orientation.x;
                gps_pose.pose.orientation.y = odom_pose_msg.pose.pose.orientation.y;
                gps_pose.pose.orientation.z = odom_pose_msg.pose.pose.orientation.z;
                gps_pose.pose.orientation.w = odom_pose_msg.pose.pose.orientation.w;
                current_pose = gps_pose;
                pub_pose -> publish(current_pose);

                path_taken.poses.push_back(current_pose);

                Eigen::Matrix4f gps_transform_mat = getTransformation(current_pose.pose);


                std::cout <<"============================================================================"<< std::endl;
                std::cout <<"=============================Gps debug==========================="<< std::endl;
                std::cout << "currrent transformation: " << std::endl;
                std::cout << gps_transform_mat << std::endl;
                std::cout <<"============================================================================"<< std::endl;

            }

            
        };

        auto geo_pose_callback = [this](const typename geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void
        {

            current_pose = *msg;
            previous_position.x() = current_pose.pose.position.x;
            previous_position.y() = current_pose.pose.position.y;
            previous_position.z() = current_pose.pose.position.z;

            pub_pose -> publish(current_pose);
            
        };

        auto cloud_callback = [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
        {
            
            header_msg.header = msg->header;

            if(!gps_available)
            {   
                RCLCPP_INFO(get_logger(), "GPS is not available - take pose from GPS");
                if(initial_state_received_)
                {
                    //define the transformation between lidar and base link
                    sensor_msgs::msg::PointCloud2 cloud_transformed_msg;

                    try{

                        tf2::TimePoint time_point = tf2::TimePoint(std::chrono::seconds(msg->header.stamp.sec) + std::chrono::nanoseconds(msg->header.stamp.nanosec));

                        const geometry_msgs::msg::TransformStamped geo_transformed_msg = tfbuffer.lookupTransform(robot_frame_id_, msg->header.frame_id, time_point);

                        tf2::doTransform(*msg, cloud_transformed_msg, geo_transformed_msg);
                        
                    }catch(tf2::TransformException &e){
                        
                        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
                        
                        return;
                    }

                    //declare a space for transformed cloud
                    pcl::PointCloud<pcl::PointXYZI>::Ptr stored_cloud(new pcl::PointCloud<pcl::PointXYZI>());

                    pcl::fromROSMsg(cloud_transformed_msg, *stored_cloud);

                    if(!initial_cloud_received_)
                    {   
                        RCLCPP_INFO(get_logger(), "create a first map");

                        pcl::PointCloud<pcl::PointXYZI>::Ptr init_cloud(new pcl::PointCloud<pcl::PointXYZI>());

                        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
                        voxel_grid.setLeafSize(vg_size_,vg_size_,vg_size_);
                        voxel_grid.setInputCloud(stored_cloud);
                        voxel_grid.filter(*init_cloud);

                        Eigen::Matrix4f transform_mat = getTransformation(current_pose.pose);

                        pcl::PointCloud<pcl::PointXYZI>::Ptr init_cloud_transformed(new pcl::PointCloud<pcl::PointXYZI>());

                        pcl::transformPointCloud(*init_cloud, *init_cloud_transformed, transform_mat);

                        registration->setInputTarget(init_cloud_transformed);

                        initial_cloud_received_ = true;
                        

                        //map
                        sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
                        pcl::toROSMsg(*init_cloud_transformed,*map_msg_ptr);

                        //map array
                        sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr(new sensor_msgs::msg::PointCloud2);
                        pcl::toROSMsg(*init_cloud,*cloud_msg_ptr);
                        lidarslam_msgs::msg::SubMap submap;
                        submap.header = msg->header;
                        submap.distance = 0;
                        submap.pose =current_pose.pose;
                        submap.cloud = *cloud_msg_ptr;
                        map_array_msg_.header = msg->header;
                        map_array_msg_.submaps.push_back(submap);

                        pub_map->publish(submap.cloud);

                        last_map_time = clock_.now();
                        
                    }

                    if(initial_cloud_received_)
                    {
                        process_cloud(stored_cloud, msg->header.stamp);
                    }

                }
            }

        };

        //=====================================================
        //initialize publisher and subscriber
        //=====================================================
        RCLCPP_INFO(get_logger(), "initialize Publishers and Subscribers");
        //subcriber
        sub_odom_pose = create_subscription<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::SensorDataQoS(), odom_callback);

        sub_cloud_input = create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic_, rclcpp::SensorDataQoS(), cloud_callback);

        sub_geo_pose = create_subscription<geometry_msgs::msg::PoseStamped>("geo_pose", rclcpp::QoS(10), geo_pose_callback);

        //Publisher
        pub_pose = create_publisher<geometry_msgs::msg::PoseStamped>("current_pose",rclcpp::QoS(10));

        pub_path = create_publisher<nav_msgs::msg::Path>("path", rclcpp::QoS(10));

        pub_ref_pose = create_publisher<geometry_msgs::msg::PoseStamped>("ref_pose", rclcpp::QoS(10));

        pub_map = create_publisher<sensor_msgs::msg::PointCloud2>("map", rclcpp::QoS(10));

        pub_map_array = create_publisher<lidarslam_msgs::msg::MapArray>("map_array", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

        RCLCPP_INFO(get_logger(), "initialization end");
    }

    //------------------------------------------------------------------
    //this function is to process the input cloud 
    //------------------------------------------------------------------
    void ScanmatchingComponent::process_cloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud_in, const rclcpp::Time stamp)
    {   
        
        if(mapping_flag && mapping_future.valid())
        {
            auto status = mapping_future.wait_for(0s);
            
            if(status == std::future_status::ready)
            {
                if(is_map_updated == true)
                {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr targeted_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(targeted_cloud));
                    
                    if(registration_method_ == "NDT_OMP")
                    {
                        registration->setInputTarget(targeted_cloud_ptr);

                    }if(registration_method_ == "GICP_OMP")
                    {
                        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_targeted_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());

                        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;

                        voxel_grid.setLeafSize(vg_size_,vg_size_,vg_size_);
                        voxel_grid.setInputCloud(targeted_cloud_ptr);
                        voxel_grid.filter(*filtered_targeted_cloud_ptr);
                        
                        registration->setInputTarget(filtered_targeted_cloud_ptr);

                    }
                    is_map_updated = false;
                }
                mapping_flag = false;
                mapping_thread.detach();
            }
        }

        ekf_complete = false;
                    
        //declare
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_in(new pcl::PointCloud<pcl::PointXYZI>());
        //declare voxel
        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
        voxel_grid.setLeafSize(vg_size_, vg_size_, vg_size_);//TODO:decalre vg sizer in param
        voxel_grid.setInputCloud(cloud_in);
        voxel_grid.filter(*filtered_cloud_in);
        //point cloud registration
        registration->setInputSource(filtered_cloud_in);

        //get the transformation matrix from current pose (current pose consist of x,y,z,qx,qy,qz)
        Eigen::Matrix4f transform_mat = getTransformation(current_pose.pose);
        //
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
        rclcpp::Clock system_clock;
            

        //allign the cloud with transform matrix
        rclcpp::Time time_align_start = system_clock.now();
    
        //##################################################
        if(use_ekf_)
        {    
            //declare
            geometry_msgs::msg::PoseStamped preallocation_pose;
            tf2::Quaternion preallocate_tf2_quat_msg, preallocate_tf2_quat_new_msg;
            Eigen::Vector3d preallocate_translation_vector;

            preallocation_pose = current_pose;

            preallocate_translation_vector.x() = preallocation_pose.pose.position.x;
            preallocate_translation_vector.y() = preallocation_pose.pose.position.y;
            preallocate_translation_vector.z() = preallocation_pose.pose.position.z;

            // Convert Eigen::Quaternion to geometry_msgs::msg::Quaternion
            geometry_msgs::msg::Quaternion preallocate_geo_quat_msg;
            
            preallocate_geo_quat_msg = preallocation_pose.pose.orientation;

            // Convert geometry_msgs::msg::Quaternion to tf2::Quaternion
            tf2::fromMsg(preallocate_geo_quat_msg, preallocate_tf2_quat_msg);

            //tf2::fromMsg(corrent_pose_stamped_.pose.orientation, quat_tf);
            double preallocate_roll, preallocate_pitch , preallocate_yaw;

            tf2::Matrix3x3(preallocate_tf2_quat_msg).getRPY(preallocate_roll, preallocate_pitch, preallocate_yaw);

            //run ekf
            ekf_scanmatching(preallocate_translation_vector, preallocate_yaw, stamp, x_out);

            preallocation_pose.pose.position.x = x_out(0,0);
            preallocation_pose.pose.position.y = x_out(1,0);
            //preallocate_yaw = x_out(6,0);

            if(neglect_roll_pitch_z_)
            {
                preallocate_tf2_quat_new_msg.setRPY(0,0,preallocate_yaw);

                preallocation_pose.pose.position.z = 0;

            }else if(!neglect_roll_pitch_z_)
            {
                preallocate_tf2_quat_new_msg.setRPY(preallocate_roll, preallocate_pitch, preallocate_yaw);
            }

            preallocation_pose.pose.orientation.x = preallocate_tf2_quat_new_msg.x();
            preallocation_pose.pose.orientation.y = preallocate_tf2_quat_new_msg.y();
            preallocation_pose.pose.orientation.z = preallocate_tf2_quat_new_msg.z();
            preallocation_pose.pose.orientation.w = preallocate_tf2_quat_new_msg.w();

            Eigen::Matrix4f transform_mat_ekf = getTransformation(preallocation_pose.pose);

            std::cout <<"============================================================================"<< std::endl;
            std::cout <<"=============================Scanmatching debug==========================="<< std::endl;
            std::cout << "initial transformation: " << std::endl;
            std::cout << transform_mat << std::endl;
            std::cout << "ekf_final transformation: " << std::endl;
            std::cout << transform_mat_ekf << std::endl;
            std::cout <<"============================================================================"<< std::endl;

            //transform_mat = transform_mat_ekf;

            current_pose = preallocation_pose;

            ekf_complete = true;

            RCLCPP_INFO(get_logger(),"EKF complete!");


        }

 
        //##################################################
        if(use_ekf_ && ekf_complete)
        {
            registration->align(*cloud_out, transform_mat);

        }else if(use_ekf_ && !ekf_complete)
        {
            RCLCPP_WARN(get_logger(),"EKF need some time!");

        }else if(!use_ekf_)
        {
            registration->align(*cloud_out, transform_mat);
        }
        

        rclcpp::Time time_align_end = system_clock.now();

        //get final tranformation
        Eigen::Matrix4f final_transformation = registration->getFinalTransformation();

        //TODOfunction getpose
        get_pose(cloud_in, final_transformation, stamp);

        //debugging
        tf2::Quaternion quat_transform;
        double roll, pitch , yaw;
        tf2::fromMsg(current_pose.pose.orientation, quat_transform);
        tf2::Matrix3x3 (quat_transform).getRPY(roll,pitch,yaw);
        u++;
        /*
        std::cout <<"============================================================================"<< std::endl;
        std::cout <<"=============================Scanmatching debug==========================="<< std::endl;
        std::cout <<"STEP:  "<< u << std::endl;
        std::cout <<"                                                                            " <<std::endl;
        std::cout << "nanoseconds: " << stamp.nanoseconds() << std::endl;
        std::cout << "align time: " << time_align_end.seconds() - time_align_start.seconds() << "s" <<std::endl;
        std::cout << "number of filtered cloud points: " << filtered_cloud_in->size() << std::endl;
        std::cout << "initial transformation: " << std::endl;
        std::cout << transform_mat << std::endl;
        std::cout << "has converged: " << registration->hasConverged() << std::endl;
        std::cout << "fitnes score: " << registration->getFitnessScore() << std::endl;
        std::cout << "final transdormation: " << std::endl;
        std::cout << final_transformation << std::endl;
        std::cout << "roll: " << roll*180/M_PI <<"deg" << " , " << "pitch: " << pitch*180/M_PI <<"deg" << " , " << "yaw: " << yaw*180/M_PI <<"deg" << std::endl;
        std::cout <<"============================================================================"<< std::endl;
        */
    }

    //------------------------------------------------------------------
    //this function is to process cloud and publish the pose 
    //------------------------------------------------------------------
    void ScanmatchingComponent::get_pose(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud_in, const Eigen::Matrix4f final_transformation, const rclcpp::Time stamp)
    {
        Eigen::Vector3d translation_vector = final_transformation.block<3,1>(0,3).cast<double>();

        Eigen::Matrix3d rotation_matrix = final_transformation.block<3,3>(0,0).cast<double>();

        //convert rotation matrix 3x3 into quaterniond qx qy qz qw
        Eigen::Quaterniond quat_value(rotation_matrix);

        // Convert Eigen::Quaternion to geometry_msgs::msg::Quaternion
        geometry_msgs::msg::Quaternion geo_quat_msg = tf2::toMsg(quat_value); 

        //declare quaternion
        tf2::Quaternion tf2_quat_msg, tf2_quat_new_msg;

        // Convert geometry_msgs::msg::Quaternion to tf2::Quaternion
        tf2::fromMsg(geo_quat_msg, tf2_quat_msg);

        //tf2::fromMsg(corrent_pose_stamped_.pose.orientation, quat_tf);
        double roll, pitch , yaw;
        tf2::Matrix3x3(tf2_quat_msg).getRPY(roll, pitch, yaw);

        //neglecting the roll and pitch or not
        if(neglect_roll_pitch_z_)
        {
            tf2_quat_new_msg.setRPY(0,0,yaw);
            translation_vector.z()=0;

        }else if(!neglect_roll_pitch_z_)
        {
            tf2_quat_new_msg.setRPY(roll,pitch,yaw);
        }

        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = stamp;
        transform_stamped.header.frame_id = global_frame_id_;
        transform_stamped.child_frame_id = robot_frame_id_;
        transform_stamped.transform.translation.x = translation_vector.x();
        transform_stamped.transform.translation.y = translation_vector.y();
        transform_stamped.transform.translation.z = translation_vector.z();
        transform_stamped.transform.rotation.x = tf2_quat_new_msg.x();
        transform_stamped.transform.rotation.y = tf2_quat_new_msg.y();
        transform_stamped.transform.rotation.z = tf2_quat_new_msg.z();
        transform_stamped.transform.rotation.w = tf2_quat_new_msg.w();
        broadcaster.sendTransform(transform_stamped);

        current_pose.header.stamp = stamp;
        current_pose.header.frame_id = global_frame_id_;
        current_pose.pose.position.x = translation_vector.x();
        current_pose.pose.position.y = translation_vector.y();
        current_pose.pose.position.z = translation_vector.z();
        current_pose.pose.orientation.x = tf2_quat_new_msg.x();
        current_pose.pose.orientation.y = tf2_quat_new_msg.y();
        current_pose.pose.orientation.z = tf2_quat_new_msg.z();
        current_pose.pose.orientation.w = tf2_quat_new_msg.w();

        pub_pose -> publish(current_pose);

        path_taken.poses.push_back(current_pose);

        pub_path->publish(path_taken);

        trans = (translation_vector - previous_position).norm();
        if(trans >= trans_for_mapupdate && !mapping_flag)
        {
            geometry_msgs::msg::PoseStamped current_pose_stamped;
            current_pose_stamped = current_pose;
            previous_position = translation_vector;
            mapping_task = std::packaged_task<void()>(
                std::bind(
                    &ScanmatchingComponent::update, this, cloud_in, 
                    final_transformation, current_pose_stamped));
            mapping_future = mapping_task.get_future();
            mapping_thread = std::thread(std::move(std::ref(mapping_task)));
            mapping_flag = true;
        }
        
    }

    void ScanmatchingComponent::update(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_in, const Eigen::Matrix4f final_transformation, const geometry_msgs::msg::PoseStamped current_pose_stamped)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setLeafSize(vg_size_map,vg_size_map,vg_size_map);
        vg.setInputCloud(cloud_in);
        vg.filter(*filtered_cloud_ptr);

        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*filtered_cloud_ptr,*transformed_cloud_ptr,final_transformation);

        targeted_cloud.clear();
        targeted_cloud += *transformed_cloud_ptr;
        int num_submaps = map_array_msg_.submaps.size();
        for(int i =0;i<num_targeted_cloud -1; i++)
        {
            if(num_submaps -1 -i < 0)
            {
                continue;
            }

            pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(map_array_msg_.submaps[num_submaps - 1 -i].cloud, *tmp_ptr);
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>());
            Eigen::Affine3d submap_affine; 
            tf2::fromMsg(map_array_msg_.submaps[num_submaps -1- i].pose, submap_affine);
            pcl::transformPointCloud(*tmp_ptr, *transformed_tmp_ptr, submap_affine.matrix());
            targeted_cloud += *transformed_tmp_ptr;

        }

        //map array
        sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr(new sensor_msgs::msg::PointCloud2);
        pcl::toROSMsg(*filtered_cloud_ptr, *cloud_msg_ptr);

        lidarslam_msgs::msg::SubMap submap;
        submap.header.frame_id = global_frame_id_;
        submap.header.stamp = current_pose_stamped.header.stamp;
        latest_distance += trans;
        submap.distance = latest_distance;
        submap.pose = current_pose_stamped.pose;
        submap.cloud = *cloud_msg_ptr;
        submap.cloud.header.frame_id = global_frame_id_;
        map_array_msg_.header.stamp = current_pose_stamped.header.stamp;
        map_array_msg_.submaps.push_back(submap);
        pub_map_array ->publish(map_array_msg_);

        is_map_updated = true;

        rclcpp::Time map_time = clock_.now();
        double dt = map_time.seconds() - last_map_time.seconds();
        if(dt>map_publish_period)
        {
            publishMap();
            last_map_time = map_time;
        }
        
    }

    //------------------------------------------------------------------
    //this function is application of extended kalman filter
    //------------------------------------------------------------------
    void ScanmatchingComponent::publishMap()
    {
        RCLCPP_INFO(get_logger(),"publish a map");

        pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new::pcl::PointCloud<pcl::PointXYZI>);

        for(auto &submap : map_array_msg_.submaps)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr submap_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_submap_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(submap.cloud, *submap_cloud_ptr);

            Eigen::Affine3d affine;
            tf2::fromMsg(submap.pose, affine);
            pcl::transformPointCloud(*submap_cloud_ptr, *transformed_submap_cloud_ptr, affine.matrix().cast<float>());

            *map_ptr += *transformed_submap_cloud_ptr;
        }

        std::cout << "number of map points: " << map_ptr->size() << std::endl;

        sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
        pcl::toROSMsg(*map_ptr, *map_msg_ptr);
        map_msg_ptr->header.frame_id = global_frame_id_;
        pub_map->publish(*map_msg_ptr);
    }


    //------------------------------------------------------------------
    //this function is application of extended kalman filter
    //------------------------------------------------------------------
    void ScanmatchingComponent::ekf_scanmatching(Eigen::Vector3d translation_vector, double yaw_in, const rclcpp::Time stamp, Eigen::Matrix<double, 6, 1> &x_out)
    {
        RCLCPP_INFO(get_logger(), "use_scanmatching");
        //devlare R variance
        var_R << pow(R_variance_,2), R_variance_, sqrt(R_variance_); //TODO declare var_R in public

        //input is the predicted scanmatching value of x and y
        double trans_pose_x = translation_vector(0);
        double trans_pose_y = translation_vector(1);
        double trans_yaw = yaw_in;

        //cummulative distace of x and y
        measurement_vector << trans_pose_x, trans_pose_y, trans_yaw; //correction state

        if(!ekf_initial_state_received_)
        {
            ekf.initialize_state(trans_pose_x, trans_pose_y, trans_yaw, init_x_out_, init_p_out_);

            x = init_x_out_;

            p = init_p_out_;

            ekf_initial_state_received_ = true;

        }
        
        if(ekf_initial_state_received_)
        {
            x_predict_in = x;
            
            p_predict_in = p;

            long double current_stamp_sensor = stamp.seconds() + (stamp.nanoseconds() *1e-9);

            if(previous_stamp_sensor == 0)
            {
                previous_stamp_sensor = current_stamp_sensor;
            }

            long double dt = current_stamp_sensor - previous_stamp_sensor;
            previous_stamp_sensor = current_stamp_sensor;

            ekf.prediction(dt, x_predict_in, p_predict_in, x_predict_out, p_predict_out);

            p_correct_in = p_predict_out;

            x_correct_in = x_predict_out;

            ekf.correction(var_R, measurement_vector, x_correct_in, p_correct_in, x_correct_out, p_correct_out);

            p = p_correct_out;

            x_out = x_correct_out;

        }

        //ekf debugging
        i++;
        /*
        std::cout <<"============================================================================"<< std::endl;
        std::cout <<"=================================ekf debugging==============================="<< std::endl;
        std::cout <<"STEP:  "<< i << std::endl;
        std::cout <<"                                                                            " <<std::endl; 
        std::cout <<"X_out:  " << x(0) << "    " <<"Y_out:  " << x(1) << "    " << "Yaw_out: " << x(2) << "     " << "V_out:   " << x(3) << std::endl;                  
        std::cout <<"P1: " <<  p(0,0) << "    " << "P2:  " << p(1,1)<< "    " << "P3:  " << p(2,2) << "    " << "P4:  " << p(3,3) << std::endl;
        std::cout <<"============================================================================"<< std::endl;
        */
    }

    Eigen::Matrix4f ScanmatchingComponent::getTransformation(const geometry_msgs::msg::Pose pose)
    {
        Eigen::Affine3d affine;
        tf2::fromMsg(pose, affine);

        Eigen::Matrix4f transform_mat_ekf =affine.matrix().cast<float>();
        return transform_mat_ekf;

    }

}

RCLCPP_COMPONENTS_REGISTER_NODE(autobin::ScanmatchingComponent)