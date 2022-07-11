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
                    
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_in(new pcl::PointCloud<pcl::PointXYZI>());

        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;

        voxel_grid.setLeafSize(vg_size_, vg_size_, vg_size_);

        voxel_grid.setInputCloud(cloud_in);

        voxel_grid.filter(*filtered_cloud_in);

        registration->setInputSource(filtered_cloud_in);

        Eigen::Matrix4f transform_mat = getTransformation(current_pose.pose);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);

        rclcpp::Clock system_clock;

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

        registration->align(*cloud_out, transform_mat);
        
        rclcpp::Time time_align_end = system_clock.now();

        Eigen::Matrix4f final_transformation = registration->getFinalTransformation();

        get_pose(cloud_in, final_transformation, stamp);
        
        if(!gps_available)//the latest pose need to run through scanmatching prediction
        {
            

        }else if(gps_available)//use the latest pose
        {
            //change current pose to transformation matrix
            Eigen::Matrix4f transform_mat = getTransformation(current_pose.pose); 



        }
    }
