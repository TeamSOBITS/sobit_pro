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
    bringup_pkg = robot_name + '_bringup'
    head_camera_name = "xtion" # 'xtion' or 'azure_kinect'  ## TODO : orbbec femt bolt??

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
                    "/clock" + "@rosgraph_msgs/msg/Clock" + "[ignition.msgs.Clock",
                    "/tf" + "@tf2_msgs/msg/TFMessage" + "[ignition.msgs.Pose_V",
                   ],
        output='screen'
    )

    world_file = os.path.join(get_package_share_directory(
        'sobit_pro_description'), 
        'worlds',
        'empty_w_physics.sdf'
    )

    return LaunchDescription([
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    os.path.join(get_package_share_directory('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py')
                ])
            ]),
            launch_arguments={
                'gz_args' : ' -r -v 4 ' + world_file,
            }.items()
        ),
        gz_bridge_node,
        # Launch Robot No. 1
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    os.path.join(get_package_share_directory(bringup_pkg),
                    'launch',
                    'robot.launch.py')
                ])
            ]),
            launch_arguments={
                'robot_name': robot_name if robot_id == 0 else robot_name + '_' + str(robot_id),
                'head_camera_name': head_camera_name,
                'enable_gz' : 'True',
                'robot_coords_x': '0',    # x 
                'robot_coords_y': '0',    # y
                'robot_coords_z': '0.01', # z
                'robot_coords_Y': '0',    # yaw
            }.items()
        ),
        # Launch Robot No. 2
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             os.path.join(get_package_share_directory(bringup_pkg),
        #             'launch',
        #             'gz_robot.launch.py')
        #         ])
        #     ]),
        #     launch_arguments={
        #         'robot_name': robot_name if robot_id == 0 else robot_name + '_' + str(robot_id),
        #         'enable_gz' : 'True',
        #         'head_camera_name': head_camera_name,
        #         'robot_coords_x': '0', # x 
        #         'robot_coords_y': '2', # y
        #         'robot_coords_z': '0.01', # z
        #         'robot_coords_Y': '0', # yaw
        #     }.items()
        # ),
    ])
