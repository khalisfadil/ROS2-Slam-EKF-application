import os

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    slam_param_dir = launch.substitutions.LaunchConfiguration(
        'slam_param_dir',
        default=os.path.join(
            get_package_share_directory('my_package'),
            'param',
            'slam.yaml'))

    slam = launch_ros.actions.Node(
        package='my_package',
        executable='my_slam_node',
        parameters=[slam_param_dir],
        remappings=[('cloud_topic','front_lidar'),('odom_topic','/odom')],
        output='screen'
        )
   
    tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['52','0','0','0','0','0.9981347984218669','-0.044','base_link','front_lidar']
        )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'slam_param_dir',
            default_value=slam_param_dir,
            description='Full path to main parameter file to load'),
        tf,
        slam,
            ])