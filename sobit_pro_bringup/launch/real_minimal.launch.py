import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    robot_name = 'sobit_pro'
    robot_id = 0
    bringup_pkg = robot_name + "_bringup"

    rviz_config = os.path.join(get_package_share_directory(bringup_pkg), 'rviz', 'real.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace=robot_name,
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    urg_config = os.path.join(get_package_share_directory(bringup_pkg), "config", "urg_node_params.yaml")
 
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([os.path.join(
                    get_package_share_directory(bringup_pkg),
                    'launch',
                    'robot.launch.py')
                ])
            ]),
            launch_arguments={
                'robot_name': robot_name if robot_id == 0 else robot_name + '_' + str(robot_id),
                'enable_gz' : 'False',
                'robot_coords_x': '0', # x 
                'robot_coords_y': '0', # y
                'robot_coords_Y': '0', # yaw
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([os.path.join(
                    get_package_share_directory('urg_node'),
                    'launch',
                    'urg.launch.py')
                ])
            ]),
            launch_arguments={
                "config_file" : urg_config,
                "use_namespace" : "true",
                "namespace" : robot_name,
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([os.path.join(
                    get_package_share_directory(bringup_pkg),
                    'launch',
                    'xtion.launch.py')
                ])
            ]),
            launch_arguments={
                'tf_prefix': robot_name if robot_id == 0 else robot_name + '_' + str(robot_id),
                'namespace': 'head_camera',
            }.items()
        ),
        rviz_node,
    ])
