from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    turtlebot_launch = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(

            os.path.join(
                get_package_share_directory(
                    'turtlebot3_gazebo'
                ),
                'launch',
                'empty_world.launch.py'
            )
        )
    )

    return LaunchDescription([

        turtlebot_launch,

        Node(
            package='robot_data',
            executable='odom_data',
            name='odom_data_node'
        ),

        Node(
            package='robot_data',
            executable='imu_data',
            name='imu_data_node'
        ),

        Node(
            package='robot_data',
            executable='marvelmind_data',
            name='marvelmind_data_node'
        ),

        Node(
            package='extended_kalman_filter',
            executable='ekf',
            name='estimation_node'
        ),

        Node(
            package='track_plotter',
            executable='track',
            name='track_plotter'
        )

    ])