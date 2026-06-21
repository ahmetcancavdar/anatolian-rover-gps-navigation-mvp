from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('anatolian_rover_mvp')
    params = os.path.join(pkg_share, 'config', 'sim_params.yaml')

    return LaunchDescription([
        Node(
            package='anatolian_rover_mvp',
            executable='mission_manager_node',
            name='mission_manager_node',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='anatolian_rover_mvp',
            executable='sim_gps_data_node',
            name='gps_data_node',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='anatolian_rover_mvp',
            executable='sim_imu9250_node',
            name='imu9250_node',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='anatolian_rover_mvp',
            executable='heading_node',
            name='heading_node',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='anatolian_rover_mvp',
            executable='navigation_node',
            name='navigation_node',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='anatolian_rover_mvp',
            executable='sim_motor_control_node',
            name='motor_control_node',
            output='screen',
            parameters=[params],
        ),
    ])
