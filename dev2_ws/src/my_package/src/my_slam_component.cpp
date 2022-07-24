#include <my_package/my_slam_component.hpp>
#include <chrono>

using namespace std::chrono_literals;

namespace autobin
{
    SlamComponent::SlamComponent(const rclcpp::NodeOptions & options)
    : Node("myslam", options),
    my_clock(RCL_ROS_TIME), 
    my_buffer(std::make_shared<rclcpp::Clock>(my_clock)), 
    my_listener(my_buffer), 
    my_broadcaster(this)
    {   
        //set up frame id
        //===============================
        declare_parameter("global_frame_id", "map");
        get_parameter("global_frame_id", global_frame_id);

        declare_parameter("robot_frame_id", "base_link");
        get_parameter("robot_frame_id", robot_frame_id);

        //set up topic
        //===============================
        declare_parameter("cloud_topic", "cloud_topic");
        get_parameter("cloud_topic", cloud_topic);

        declare_parameter("odom_topic", "odom_topic");
        get_parameter("odom_topic", odom_topic);

        //set up mapping parameter
        //==============================
        declare_parameter("registration_method", "NDT_OMP");
        get_parameter("registration_method", registration_method);

        declare_parameter("trans_for_map_update", 1.0);
        get_parameter("trans_for_mapudpate", trans_for_map_update);

        declare_parameter("map_publish_period", 1.0);
        get_parameter("map_publish_period", map_publish_period);

        declare_parameter("num_targeted_cloud", 200);
        get_parameter("num_targeted_cloud", num_targeted_cloud);

        //set up parameter for NDT
        //==============================
        declare_parameter("ndt_resolution", 1.0);
        get_parameter("ndt_resolution", ndt_resolution);

        declare_parameter("ndt_num_threads", 8);
        get_parameter("ndt_num_threads", ndt_num_threads);

        declare_parameter("neighborhood_search_method", "DIRECT1");
        get_parameter("neighborhood_search_method", neighborhood_search_method);

        //set up parameter for ICP
        //==============================
        declare_parameter("gicp_corr_dist_threshold", 5.0);
        get_parameter("gicp_corr_dist_threshold", gicp_corr_dist_threshold);

        //set up parameter for Voxel Grid
        //==============================
        declare_parameter("vg_size", 0.1);
        get_parameter("vg_size", vg_size);

        declare_parameter("vg_size_map", 0.1);
        get_parameter("vg_size_map", vg_size_map);

        //parameter for Kalman filtering
        //===============================
        declare_parameter("R_variance", 100.0);
        get_parameter("R_variance", R_variance);

        declare_parameter("Kalman_Filter_model", "CHCA");
        get_parameter("Kalman_Filter_model", Kalman_Filter_model);

        if(Kalman_Filter_model == "CHCA")
        {
            declare_parameter("num_state_space", 8);
            get_parameter("num_state_space", num_state_space);

        }else if(Kalman_Filter_model == "CHCV")
        {
            declare_parameter("num_state_space", 6);
            get_parameter("num_state_space", num_state_space);
        }else{

            std::cout << "Model is incorrectly define, set up the model to default" << std::endl;
            std::cout << ".................." << std::endl;
            std::cout << "Model set to 'CHCA model'" << std::endl;
            Kalman_Filter_model = "CHCA";
            declare_parameter("num_state_space", 8);
            get_parameter("num_state_space", num_state_space);

        }

        //parameter for Bool application
        //================================
        declare_parameter("use_ekf", false);
        get_parameter("use_ekf", use_ekf);

        declare_parameter("neglect_roll_pitch_z", false);
        get_parameter("neglect_roll_pitch_z", neglect_roll_pitch_z);

        //parameter statement
        //================================
        std::cout << "==================================================" << std::endl;
        std::cout << "Initializing all parameter" << std::endl;
        std::cout << "==================================================" << std::endl;
        std::cout << "Registration method: " << registration_method << std::endl;
        std::cout << "Voxel leaf size [m]: " << vg_size << std::endl;
        std::cout << "Voxel leaf size mapping [m]: " << vg_size_map << std::endl;
        std::cout << "Kalman Filter model: " << Kalman_Filter_model << std::endl;
        std::cout << "Scan method model: " << neighborhood_search_method << std::endl;
        std::cout << "Number state space: " << num_state_space << std::endl;
        std::cout << "NDT Resolution [m]: " << ndt_resolution << std::endl;
        std::cout << "NDT number of threads:" << ndt_num_threads << std::endl;
        std::cout << "GICP correlation distance threshold [m]: " << gicp_corr_dist_threshold << std::endl;
        std::cout << "Use Kalman Filter:" << std::boolalpha << use_ekf << std::endl;
        std::cout << "Neglect roll, pitch, z:" << std::boolalpha << neglect_roll_pitch_z << std::endl;
        std::cout << "==================================================" << std::endl;

        //delcare msg header
        //================================
        path_taken.header.frame_id = global_frame_id;
        map_array_msg.header.frame_id = global_frame_id;
        map_array_msg.cloud_coordinate = map_array_msg.LOCAL;
        //header_msg.header.stamp = rclcpp::Clock().now();

        //parameter for registration method
        //================================
        if(registration_method == "NDT_OMP")
        {
            pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
        
            ndt_omp->setResolution(ndt_resolution);
            ndt_omp->setTransformationEpsilon(0.01);
            ndt_omp->setNumThreads(ndt_num_threads);

            if(neighborhood_search_method == "DIRECT1")
            {
                ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT1);

            }else if(neighborhood_search_method == "DIRECT7")
            {
                ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);

            }else if(neighborhood_search_method == "KDTREE")
            {
                ndt_omp->setNeighborhoodSearchMethod(pclomp::KDTREE);

            }else{
                
                std::cout << "Search method is incorrectly define, set up the method to default" << std::endl;
                std::cout << ".................." << std::endl;
                std::cout << "Method set up to 'DIRECT1'" << std::endl;
                ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT1);
            }

            registration = ndt_omp;

        }else if(registration_method == "GICP_OMP")
        {
            pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr gicp_omp(new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
            gicp_omp->setMaxCorrespondenceDistance(gicp_corr_dist_threshold);
            gicp_omp->setTransformationEpsilon(0.01);
            registration = gicp_omp;
        }

        //call the odom msg
        //================================
        auto odom_callback = [this](const typename nav_msgs::msg::Odometry::SharedPtr msg) -> void
        {
            //stating the condition of GPS-able and GPS-denied environment
            //================================
            step++;
            std::cout << "Step: " << step << std::endl;
            if(step <= 30 )
            {
                gps_available = true;
            }else{
                gps_available = false;
            }

            //set up the initial position and orientation
            //================================
            if(!initial_state_received)
            {
                odom_pose_msg = *msg;

                auto geo_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();

                geo_msg->header.frame_id= global_frame_id;
                geo_msg->header.stamp = header_msg.header.stamp;
                geo_msg->pose.position.x = 0;
                geo_msg->pose.position.y = 0;
                geo_msg->pose.position.z = 0;
                geo_msg->pose.orientation.x = odom_pose_msg.pose.pose.orientation.x;
                geo_msg->pose.orientation.y = odom_pose_msg.pose.pose.orientation.y;
                geo_msg->pose.orientation.z = -1 * odom_pose_msg.pose.pose.orientation.z;
                geo_msg->pose.orientation.w = odom_pose_msg.pose.pose.orientation.w;

                current_pose = *geo_msg;

                //pub_pose -> publish(current_pose);

                //path_taken.poses.push_back(current_pose);

                initial_state_received = true;

            }
            
            
            odom_ref_pose_msg = *msg;

            if(previous_position_x == 0 && previous_position_y == 0 && previous_position_z == 0)
            {
                previous_position_x = odom_ref_pose_msg.pose.pose.position.x;
                previous_position_y = odom_ref_pose_msg.pose.pose.position.y;
                previous_position_z = odom_ref_pose_msg.pose.pose.position.z;
            }

            diff_pose_x = odom_ref_pose_msg.pose.pose.position.x - previous_position_x;
            diff_pose_y = odom_ref_pose_msg.pose.pose.position.y - previous_position_y;
            diff_pose_z = odom_ref_pose_msg.pose.pose.position.z - previous_position_z;

            pose_x += diff_pose_x;
            pose_y += diff_pose_y;
            pose_z += diff_pose_z;

            previous_position_x = odom_ref_pose_msg.pose.pose.position.x;
            previous_position_y = odom_ref_pose_msg.pose.pose.position.y;
            previous_position_z = odom_ref_pose_msg.pose.pose.position.z;

            ref_pose.header = header_msg.header;
            ref_pose.header.frame_id = global_frame_id;
            ref_pose.header.stamp = header_msg.header.stamp;
            ref_pose.pose.position.x = pose_x;
            ref_pose.pose.position.y = pose_y;
            ref_pose.pose.position.z = pose_z;
            ref_pose.pose.orientation.x = odom_ref_pose_msg.pose.pose.orientation.x;
            ref_pose.pose.orientation.y = odom_ref_pose_msg.pose.pose.orientation.y;
            ref_pose.pose.orientation.z = -1 * odom_ref_pose_msg.pose.pose.orientation.z;
            ref_pose.pose.orientation.w = odom_ref_pose_msg.pose.pose.orientation.w;

            //publish at this time frame
            //================================
            pub_ref_pose -> publish(ref_pose);
            pub_pose -> publish(current_pose);
            path_taken.poses.push_back(current_pose);
            pub_path->publish(path_taken);

            if(Kalman_Filter_model == "CHCA")
            {
                pub_ekf_chca -> publish(kf_msg_chca);

            }else if(Kalman_Filter_model == "CHCV")
            {
                pub_ekf_chcv -> publish(kf_msg_chcv);
            }

            //take position parameter from gps info
            //================================
            if (gps_available)
            {
                gps_pose.header.frame_id = global_frame_id;
                gps_pose.header.stamp = header_msg.header.stamp;
                gps_pose.pose.position.x = pose_x;
                gps_pose.pose.position.y = pose_y;
                gps_pose.pose.position.z = pose_z;
                gps_pose.pose.orientation.x = odom_ref_pose_msg.pose.pose.orientation.x;
                gps_pose.pose.orientation.y = odom_ref_pose_msg.pose.pose.orientation.y;
                gps_pose.pose.orientation.z = -1 * odom_ref_pose_msg.pose.pose.orientation.z;
                gps_pose.pose.orientation.w = odom_ref_pose_msg.pose.pose.orientation.w;

                current_pose = gps_pose;

                //pub_pose -> publish(current_pose);

                //path_taken.poses.push_back(current_pose);

            }


            
        };
        /*
        //call the cloud msg
        //================================
        auto geo_pose_callback = [this](const typename geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void
        {

            current_pose = *msg;
            previous_position.x() = current_pose.pose.position.x;
            previous_position.y() = current_pose.pose.position.y;
            previous_position.z() = current_pose.pose.position.z;

            pub_pose -> publish(current_pose);
            
        };
        */

        //call the cloud msg
        //================================
        auto cloud_callback = [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
        {
            //header_msg.header.stamp = msg->header.stamp;
            header_msg.header.stamp = rclcpp::Clock().now();

            //initialize cloud msg
            //================================
            if(initial_state_received)
            {
                sensor_msgs::msg::PointCloud2 cloud_transformed_msg;

                try{

                    tf2::TimePoint time_point = tf2::TimePoint(std::chrono::seconds(header_msg.header.stamp.sec) + std::chrono::nanoseconds(header_msg.header.stamp.nanosec));
                    const geometry_msgs::msg::TransformStamped geo_transformed_msg = my_buffer.lookupTransform(robot_frame_id, msg->header.frame_id, time_point);
                    tf2::doTransform(*msg, cloud_transformed_msg, geo_transformed_msg);

                }catch(tf2::TransformException &e){
                    
                    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
                    
                    return;
                }

                pcl::PointCloud<pcl::PointXYZI>::Ptr stored_cloud(new pcl::PointCloud<pcl::PointXYZI>());
                pcl::fromROSMsg(cloud_transformed_msg, *stored_cloud);

                if(!initial_cloud_received)
                {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr init_cloud(new pcl::PointCloud<pcl::PointXYZI>());
                    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;

                    voxel_grid.setLeafSize(vg_size, vg_size, vg_size);
                    voxel_grid.setInputCloud(stored_cloud);
                    voxel_grid.filter(*init_cloud);

                    Eigen::Matrix4f transform_mat = getTransformation(current_pose.pose);

                    pcl::PointCloud<pcl::PointXYZI>::Ptr init_cloud_transformed(new pcl::PointCloud<pcl::PointXYZI>());
                    pcl::transformPointCloud(*init_cloud, *init_cloud_transformed, transform_mat);

                    registration->setInputTarget(init_cloud_transformed);

                    initial_cloud_received = true;

                    //map
                    //================================
                    sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
                    pcl::toROSMsg(*init_cloud_transformed,*map_msg_ptr);

                    //map array
                    //================================
                    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr(new sensor_msgs::msg::PointCloud2);
                    pcl::toROSMsg(*init_cloud,*cloud_msg_ptr);

                    lidarslam_msgs::msg::SubMap submap;
                    submap.header = msg->header;
                    submap.distance = 0;
                    submap.pose = current_pose.pose;
                    submap.cloud = *cloud_msg_ptr;

                    map_array_msg.header = msg->header;
                    map_array_msg.submaps.push_back(submap);

                    pub_map->publish(submap.cloud);

                    last_map_time = my_clock.now();
                }

                if(initial_cloud_received)
                {
                    process_cloud(stored_cloud, msg->header.stamp);
                    //process_cloud(stored_cloud, header_msg.header.stamp);
                }
            }
        };

        //initialize Subcriber
        //================================
        sub_odom_pose = create_subscription<nav_msgs::msg::Odometry>(odom_topic, rclcpp::SensorDataQoS(), odom_callback);

        sub_cloud_input = create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic, rclcpp::SensorDataQoS(), cloud_callback);

        //sub_geo_pose = create_subscription<geometry_msgs::msg::PoseStamped>("geo_pose", rclcpp::QoS(10), geo_pose_callback);
        //initialize Publisher
        //================================
        pub_pose = create_publisher<geometry_msgs::msg::PoseStamped>("current_pose",rclcpp::QoS(10));

        pub_path = create_publisher<nav_msgs::msg::Path>("path", rclcpp::QoS(10));

        pub_ref_pose = create_publisher<geometry_msgs::msg::PoseStamped>("ref_pose", rclcpp::QoS(10));

        pub_map = create_publisher<sensor_msgs::msg::PointCloud2>("map", rclcpp::QoS(10));

        pub_map_array = create_publisher<lidarslam_msgs::msg::MapArray>("map_array", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

        if(Kalman_Filter_model == "CHCA")
        {
            pub_ekf_chca = create_publisher<kalman_msgs::msg::Chca>("kalman_chca", rclcpp::QoS(10));
        
        }else if(Kalman_Filter_model == "CHCV")
        {
            pub_ekf_chcv = create_publisher<kalman_msgs::msg::Chcv>("kalman_chcv", rclcpp::QoS(10));
        }
    }

    //function process process_cloud
    //================================
    void SlamComponent::process_cloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud_in, const rclcpp::Time stamp)
    {
        //define the mapping start
        //================================
        if(mapping_flag && mapping_future.valid())
        {
            auto status = mapping_future.wait_for(0s);
    
            if(status == std::future_status::ready)
            {
                if(is_map_updated == true)
                {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr targeted_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(targeted_cloud));
                    
                    if(registration_method == "NDT_OMP")
                    {
                        registration->setInputTarget(targeted_cloud_ptr);

                    }if(registration_method == "GICP_OMP")
                    {
                        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_targeted_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
                        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;

                        voxel_grid.setLeafSize(vg_size,vg_size,vg_size);
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

        //define the transformation matrix for gps-able or gps denied environment
        //================================
        if(gps_available)
        {   
            RCLCPP_INFO(get_logger(), "::: GPS available ::: take pose from GPS :::");

            Eigen::Matrix4f gps_transform_mat = getTransformation(current_pose.pose);
            Eigen::Vector3d gps_translation_vector = gps_transform_mat.block<3,1>(0,3).cast<double>();
            Eigen::Matrix3d gps_rotation_matrix = gps_transform_mat.block<3,3>(0,0).cast<double>();

            Eigen::Quaterniond gps_quat(gps_rotation_matrix);
            geometry_msgs::msg::Quaternion gps_geo_quat = tf2::toMsg(gps_quat);
            tf2::Quaternion gps_tf2_quat, gps_tf2_quat_new;
            tf2::fromMsg(gps_geo_quat, gps_tf2_quat);

            double gps_roll, gps_pitch, gps_yaw;
            tf2::Matrix3x3(gps_tf2_quat).getRPY(gps_roll, gps_pitch, gps_yaw);

            if(Kalman_Filter_model == "CHCA")
            {
                ekf_scanmatching_CHCA(gps_translation_vector, gps_yaw, stamp, state_chca_out);
            
            }else if(Kalman_Filter_model == "CHCV")
            {
                ekf_scanmatching_CHCV(gps_translation_vector, gps_yaw, stamp, state_chcv_out);
            } 

            std::cout <<"============================================================================"<< std::endl;
            std::cout <<"=============================Gps debug==========================="<< std::endl;
            std::cout << "Currrent transformation matrix [GPS]: " << std::endl;
            std::cout << gps_transform_mat << std::endl;
            std::cout <<"============================================================================"<< std::endl;

            get_pose(cloud_in, gps_transform_mat, stamp);

        }else if(!gps_available)
        {
            
            RCLCPP_INFO(get_logger(), "::: GPS NOT available ::: take pose from Scanmatching :::");

            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;

            voxel_grid.setLeafSize(vg_size, vg_size, vg_size);
            voxel_grid.setInputCloud(cloud_in);
            voxel_grid.filter(*filtered_cloud_in);

            registration->setInputSource(filtered_cloud_in);

            Eigen::Matrix4f pre_scan_transform_mat= getTransformation(current_pose.pose);

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);

            rclcpp::Clock system_clock;
            rclcpp::Time time_align_start = system_clock.now();
            registration->align(*cloud_out, pre_scan_transform_mat);
            rclcpp::Time time_align_end = system_clock.now();

            Eigen::Matrix4f scan_transform_mat = registration->getFinalTransformation();
            Eigen::Vector3d scan_translation_vector = scan_transform_mat.block<3,1>(0,3).cast<double>();
            Eigen::Matrix3d scan_rotation_matrix = scan_transform_mat.block<3,3>(0,0).cast<double>();
            Eigen::Quaterniond scan_quat(scan_rotation_matrix);
            geometry_msgs::msg::Quaternion scan_geo_quat = tf2::toMsg(scan_quat);
            tf2::Quaternion scan_tf2_quat, scan_tf2_quat_new;
            tf2::fromMsg(scan_geo_quat, scan_tf2_quat);

            double scan_roll, scan_pitch, scan_yaw;
            tf2::Matrix3x3(scan_tf2_quat).getRPY(scan_roll, scan_pitch, scan_yaw);

            if(Kalman_Filter_model == "CHCA")
            {
                ekf_scanmatching_CHCA(scan_translation_vector, scan_yaw, stamp, state_chca_out);
            
            }else if(Kalman_Filter_model == "CHCV")
            {
                ekf_scanmatching_CHCV(scan_translation_vector, scan_yaw, stamp, state_chcv_out);
            } 

            if(use_ekf)
            {

                if(Kalman_Filter_model == "CHCA")
                {
                    scan_translation_vector.x() = state_chca_out(0);
                    scan_translation_vector.y() = state_chca_out(1);
                    scan_yaw = state_chca_out(6);
                
                }else if(Kalman_Filter_model == "CHCV")
                {
                    scan_translation_vector.x() = state_chcv_out(0);
                    scan_translation_vector.y() = state_chcv_out(1);
                    scan_yaw = state_chcv_out(4);
                } 
            }

            if(neglect_roll_pitch_z)
            {
                scan_tf2_quat_new.setRPY(0,0,scan_yaw);
                scan_translation_vector.z() = 0;

            }else if(!neglect_roll_pitch_z)
            {
                scan_tf2_quat_new.setRPY(scan_roll,scan_pitch,scan_yaw);
            }

            geometry_msgs::msg::Quaternion final_scan_geo_quat = tf2::toMsg(scan_tf2_quat_new);
            Eigen::Quaterniond final_scan_eigen_quat;
            tf2::fromMsg(final_scan_geo_quat, final_scan_eigen_quat);
            Eigen::Matrix3d final_scan_rotation_mat_d = final_scan_eigen_quat.toRotationMatrix();
            Eigen::Matrix3f final_scan_rotation_mat = final_scan_rotation_mat_d.cast<float>();
            Eigen::Vector3f final_scan_translation_vector = scan_translation_vector.cast<float>();
            Eigen::Matrix4f final_scan_transform_mat = Eigen::Matrix4f::Identity();
            final_scan_transform_mat.block<3,3>(0,0) = final_scan_rotation_mat;
            final_scan_transform_mat.block<3,1>(0,3) = final_scan_translation_vector;

            std::cout <<"============================================================================"<< std::endl;
            std::cout <<"=============================Scan debug==========================="<< std::endl;
            std::cout << "Pre transformation matrix [SCAN]: " << std::endl;
            std::cout << scan_transform_mat << std::endl;
            std::cout << "Currrent transformation matrix [SCAN]: " << std::endl;
            std::cout << final_scan_transform_mat << std::endl;
            std::cout <<"============================================================================"<< std::endl;

            get_pose(cloud_in, final_scan_transform_mat, stamp);
        }
    }
    
    //function to publish the pose
    //================================
    void SlamComponent::get_pose(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud_in, const Eigen::Matrix4f final_transformation, const rclcpp::Time stamp)
    {
        Eigen::Vector3d translation_vector = final_transformation.block<3,1>(0,3).cast<double>();
        Eigen::Matrix3d rotation_matrix = final_transformation.block<3,3>(0,0).cast<double>();
        Eigen::Quaterniond quat_value(rotation_matrix);

        geometry_msgs::msg::Quaternion geo_quat_msg = tf2::toMsg(quat_value); 

        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = stamp;
        transform_stamped.header.frame_id = global_frame_id;
        transform_stamped.child_frame_id = robot_frame_id;
        transform_stamped.transform.translation.x = translation_vector.x();
        transform_stamped.transform.translation.y = translation_vector.y();
        transform_stamped.transform.translation.z = translation_vector.z();
        transform_stamped.transform.rotation = geo_quat_msg;
        my_broadcaster.sendTransform(transform_stamped);

        current_pose.header.stamp = stamp;
        current_pose.header.frame_id = global_frame_id;
        current_pose.pose.position.x = translation_vector.x();
        current_pose.pose.position.y = translation_vector.y();
        current_pose.pose.position.z = translation_vector.z();
        current_pose.pose.orientation = geo_quat_msg;

        //pub_pose -> publish(current_pose);

        //path_taken.poses.push_back(current_pose);

        //pub_path->publish(path_taken);

        trans = (translation_vector - previous_position).norm();
        
        if(trans >= trans_for_map_update && !mapping_flag)
        {
            geometry_msgs::msg::PoseStamped current_pose_stamped;
            current_pose_stamped = current_pose;
            previous_position = translation_vector;

            mapping_task = std::packaged_task<void()>(std::bind(&SlamComponent::update, this, cloud_in, final_transformation, current_pose_stamped));
            mapping_future = mapping_task.get_future();
            mapping_thread = std::thread(std::move(std::ref(mapping_task)));
            mapping_flag = true;
        }
        
    }

    //function to update the map
    //================================
    void SlamComponent::update(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_in, const Eigen::Matrix4f final_transformation, const geometry_msgs::msg::PoseStamped current_pose_stamped)
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

        int num_submaps = map_array_msg.submaps.size();

        for(int i =0;i<num_targeted_cloud -1; i++)
        {
            if(num_submaps -1 -i < 0)
            {
                continue;
            }

            pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(map_array_msg.submaps[num_submaps - 1 -i].cloud, *tmp_ptr);
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>());

            Eigen::Affine3d submap_affine; 
            tf2::fromMsg(map_array_msg.submaps[num_submaps -1- i].pose, submap_affine);
            pcl::transformPointCloud(*tmp_ptr, *transformed_tmp_ptr, submap_affine.matrix());
            targeted_cloud += *transformed_tmp_ptr;
        }

        sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr(new sensor_msgs::msg::PointCloud2);

        pcl::toROSMsg(*filtered_cloud_ptr, *cloud_msg_ptr);

        lidarslam_msgs::msg::SubMap submap;

        submap.header.frame_id = global_frame_id;
        submap.header.stamp = current_pose_stamped.header.stamp;
        latest_distance += trans;
        submap.distance = latest_distance;
        submap.pose = current_pose_stamped.pose;
        submap.cloud = *cloud_msg_ptr;
        submap.cloud.header.frame_id = global_frame_id;
        map_array_msg.header.stamp = current_pose_stamped.header.stamp;
        map_array_msg.submaps.push_back(submap);
        pub_map_array ->publish(map_array_msg);

        is_map_updated = true;

        rclcpp::Time map_time = my_clock.now();

        double dt = map_time.seconds() - last_map_time.seconds();

        if(dt>map_publish_period)
        {
            publishMap();
            last_map_time = map_time;
        }
    }

    //function publish the map
    //================================
    void SlamComponent::publishMap()
    {
        RCLCPP_INFO(get_logger(),"publish a map");

        pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new::pcl::PointCloud<pcl::PointXYZI>);

        for(auto &submap : map_array_msg.submaps)
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
        map_msg_ptr->header.frame_id = global_frame_id;
        pub_map->publish(*map_msg_ptr);
    }

    //function for application of extended kalman filter model CHCA
    //================================
    void SlamComponent::ekf_scanmatching_CHCA(Eigen::Vector3d translation_vector, double yaw_in, const rclcpp::Time stamp, Eigen::Matrix<double, 8, 1> &x_chca_out)
    {

        var_R << pow(R_variance,2), R_variance, sqrt(R_variance); 

        double trans_pose_x = translation_vector(0);
        double trans_pose_y = translation_vector(1);
        double trans_yaw = yaw_in;

        measurement_vector << trans_pose_x, trans_pose_y, trans_yaw; //correction state

        if(!ekf_initial_state_received)
        {
            ekf_chca.initial_state(trans_pose_x, trans_pose_y, trans_yaw, init_x_chca_out, init_p_chca_out);

            x_chca = init_x_chca_out;
            p_chca = init_p_chca_out;
            
            ekf_initial_state_received = true;
        }
        
        if(ekf_initial_state_received)
        {
            x_predict_chca_in = x_chca;
            p_predict_chca_in = p_chca;

            long double current_stamp_sensor = stamp.seconds() + (stamp.nanoseconds() *1e-9);

            if(previous_stamp_sensor == 0)
            {
                previous_stamp_sensor = current_stamp_sensor;
            }

            long double dt = current_stamp_sensor - previous_stamp_sensor;
            previous_stamp_sensor = current_stamp_sensor;

            ekf_chca.prediction(dt, x_predict_chca_in, p_predict_chca_in, x_predict_chca_out, p_predict_chca_out);

            p_correct_chca_in = p_predict_chca_out;
            x_correct_chca_in = x_predict_chca_out;

            ekf_chca.correction(var_R, measurement_vector, x_correct_chca_in, p_correct_chca_in, x_correct_chca_out, p_correct_chca_out);

            p_chca = p_correct_chca_out;
            x_chca = x_correct_chca_out;

            x_chca_out = x_chca;


            //kalman_msgs::msg::Chca kf_msg_chca;
            kf_msg_chca.header.stamp = stamp;
            kf_msg_chca.header.frame_id = global_frame_id;
            kf_msg_chca.refposex = translation_vector(0);
            kf_msg_chca.refposey = translation_vector(1);
            kf_msg_chca.x = x_chca(0);
            kf_msg_chca.y = x_chca(1);
            kf_msg_chca.vx = x_chca(2);
            kf_msg_chca.vy = x_chca(3);
            kf_msg_chca.ax = x_chca(4);
            kf_msg_chca.ay = x_chca(5);
            kf_msg_chca.p = x_chca(6);
            kf_msg_chca.vp = x_chca(7);
            kf_msg_chca.p1 = p_chca(0,0);
            kf_msg_chca.p2 = p_chca(1,1);
            kf_msg_chca.p3 = p_chca(2,2);
            kf_msg_chca.p4 = p_chca(3,3);
            kf_msg_chca.p5 = p_chca(4,4);
            kf_msg_chca.p6 = p_chca(5,5);
            kf_msg_chca.p7 = p_chca(6,6);
            kf_msg_chca.p8 = p_chca(7,7);

            //pub_ekf_chca -> publish(kf_msg);
                
        }
    }

    //function for application of extended kalman filter model CHCV
    //================================
    void SlamComponent::ekf_scanmatching_CHCV(Eigen::Vector3d translation_vector, double yaw_in, const rclcpp::Time stamp, Eigen::Matrix<double, 6, 1> &x_chcv_out)
    {

        var_R << pow(R_variance,2), R_variance, sqrt(R_variance); 

        double trans_pose_x = translation_vector(0);
        double trans_pose_y = translation_vector(1);
        double trans_yaw = yaw_in;

        measurement_vector << trans_pose_x, trans_pose_y, trans_yaw; //correction state

        if(!ekf_initial_state_received)
        {
            ekf_chcv.initial_state(trans_pose_x, trans_pose_y, trans_yaw, init_x_chcv_out, init_p_chcv_out);

            x_chcv = init_x_chcv_out;
            p_chcv = init_p_chcv_out;
            
            ekf_initial_state_received = true;
        }
        
        if(ekf_initial_state_received)
        {
            x_predict_chcv_in = x_chcv;
            p_predict_chcv_in = p_chcv;

            long double current_stamp_sensor = stamp.seconds() + (stamp.nanoseconds() *1e-9);

            if(previous_stamp_sensor == 0)
            {
                previous_stamp_sensor = current_stamp_sensor;
            }

            long double dt = current_stamp_sensor - previous_stamp_sensor;
            previous_stamp_sensor = current_stamp_sensor;

            ekf_chcv.prediction(dt, x_predict_chcv_in, p_predict_chcv_in, x_predict_chcv_out, p_predict_chcv_out);

            p_correct_chcv_in = p_predict_chcv_out;
            x_correct_chcv_in = x_predict_chcv_out;

            ekf_chcv.correction(var_R, measurement_vector, x_correct_chcv_in, p_correct_chcv_in, x_correct_chcv_out, p_correct_chcv_out);

            p_chcv = p_correct_chcv_out;
            x_chcv = x_correct_chcv_out;

            x_chcv_out = x_chcv;

            //kalman_msgs::msg::Chcv kf_msg;
            kf_msg_chcv.header.stamp = stamp;
            kf_msg_chcv.header.frame_id = global_frame_id;
            kf_msg_chcv.refposex = translation_vector(0);
            kf_msg_chcv.refposey = translation_vector(1);
            kf_msg_chcv.x = x_chcv(0);
            kf_msg_chcv.y = x_chcv(1);
            kf_msg_chcv.vx = x_chcv(2);
            kf_msg_chcv.vy = x_chcv(3);
            kf_msg_chcv.p = x_chcv(4);
            kf_msg_chcv.vp = x_chcv(5);
            kf_msg_chcv.p1 = p_chcv(0,0);
            kf_msg_chcv.p2 = p_chcv(1,1);
            kf_msg_chcv.p3 = p_chcv(2,2);
            kf_msg_chcv.p4 = p_chcv(3,3);
            kf_msg_chcv.p5 = p_chcv(4,4);
            kf_msg_chcv.p6 = p_chcv(5,5);

            //pub_ekf_chcv -> publish(kf_msg);
        }
    }

    

    //get transformation matrix of current pose
    //================================
    Eigen::Matrix4f SlamComponent::getTransformation(const geometry_msgs::msg::Pose pose)
    {
        Eigen::Affine3d affine;
        tf2::fromMsg(pose, affine);
        Eigen::Matrix4f transform_mat =affine.matrix().cast<float>();
        return transform_mat;
    } 

}

RCLCPP_COMPONENTS_REGISTER_NODE(autobin::SlamComponent)