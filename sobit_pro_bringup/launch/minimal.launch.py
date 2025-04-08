import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

import yaml 
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.actions import Node

def generate_launch_description():
    robot_name = 'sobit_pro'
    bringup_pkg = robot_name + "_bringup"

    rviz_config = os.path.join(get_package_share_directory(
        bringup_pkg), "rviz", "real.rviz")
    
    urg_config = os.path.join(get_package_share_directory(
        bringup_pkg), "config", "urg_node_params.yaml")
    
    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
        Node(
            package="sobit_pro_control",
            executable="sobit_pro_main",
            name="sobit_pro_main",
            output="screen",
            namespace="sobit_pro"
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([os.path.join(
                    get_package_share_directory('sobit_pro_bringup'),
                    'launch',
                    'robot.launch.py')
                ])

            ]),
            launch_arguments={
                'robot_name': 'sobit_pro',
                'robot_coords_x': '0', # x 
                'robot_coords_y': '0', # y
                'robot_coords_Y': '0', # yaw
            }.items()
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([os.path.join(
        #             get_package_share_directory('sobit_pro_bringup'),
        #             'launch',
        #             'realsense_bringup.launch.py')
        #         ])

        #     ]),
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('azure_kinect_ros_driver'),
                    'launch',
                    'driver.launch.py'
                ])

            ]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    get_package_share_directory('urg_node'),
                    'launch',
                    'urg.launch.py'
                ])
            ]),
            launch_arguments={
                "config_file" : urg_config
            }.items()
        )
    ])