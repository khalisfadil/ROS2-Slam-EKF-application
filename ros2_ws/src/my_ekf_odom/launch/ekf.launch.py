import os

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ekf_param_dir = launch.substitutions.LaunchConfiguration(
        'ekf_param_dir',
        default=os.path.join(
            get_package_share_directory('my_ekf_odom'),
            'param',
            'ekf.yaml'))

    ekf = launch_ros.actions.Node(
        package='my_ekf_odom',
        executable='ekf_odom_node',
        parameters=[ekf_param_dir],
        remappings=[('odom_pose', '/odom')],
        output='screen'
        )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'ekf_param_dir',
            default_value=ekf_param_dir,
            description='Full path to ekf parameter file to load'),
        ekf,
            ])
