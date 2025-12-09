from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name = 'sobit_pro'
    head_camera_name = "xtion" # 'xtion' or 'azure_kinect' or 'femtobolt
    only_mobile_base_hardware = False # You use only mobile base to URG serial

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('sobit_pro_bringup'),
                    'launch',
                    'robot.launch.py'
                ])
            ]),
            launch_arguments={
                'robot_name': robot_name,
                'head_camera_name': head_camera_name,
                'enable_mb' : 'True',
                'enable_arm' : 'True',
                'enable_head' : 'True',
                'enable_gz' : 'False',
                'use_serial_urg': 'True' if (only_mobile_base_hardware) else 'False',
            }.items()
        ),
    ])
