import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name = 'sobit_pro'
    robot_id = 0
    head_camera_name = "xtion" # 'xtion' or 'azure_kinect' or 'femtobolt
    world_model = 'rcjo2025_arena' # empty, wrs, small_house, rcjo2025_arena

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
                    "/clock" + "@rosgraph_msgs/msg/Clock" + "[gz.msgs.Clock",
                    "/tf" + "@tf2_msgs/msg/TFMessage" + "[gz.msgs.Pose_V",
                   ],
        output='screen'
    )

    world_file = ''
    if world_model == 'empty':
        world_file = os.path.join(get_package_share_directory(
            'sobit_pro_description'), 
            'worlds',
            'empty_w_physics.sdf'
        )
    elif world_model == 'wrs':
        world_file = os.path.join(get_package_share_directory(
            'tmc_wrs_gz_worlds'), 
            'worlds',
            'wrs2020.world.xacro'
        )
    elif world_model == 'small_house':
        world_file = os.path.join(get_package_share_directory(
            'aws_small_house_world'), 
            'worlds',
            'small_house.world'
        )
    elif world_model == 'rcjo2025_arena':
        world_file = os.path.join(get_package_share_directory(
            'sobits_gazebo_worlds'), 
            'worlds',
            'rcjo2025_arena.world.xacro'
        )

    rviz_config = PathJoinSubstitution([
            FindPackageShare('sobit_pro_bringup'),
            'rviz',
            'gazebo.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
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
                    os.path.join(get_package_share_directory('sobit_pro_bringup'),
                    'launch',
                    'robot.launch.py')
                ])
            ]),
            launch_arguments={
                'robot_name': robot_name if robot_id == 0 else robot_name + '_' + str(robot_id),
                'head_camera_name'        : head_camera_name,
                'enable_mb'               : 'True',
                'enable_arm'              : 'True',
                'enable_head'             : 'True',
                'robot_coords_x'          : '-5.5', # x 
                'robot_coords_y'          : '1.5',  # y
                'robot_coords_z'          : '0.01', # z
                'robot_coords_Y'          : '0.0',  # yaw
                'enable_gz_lidar'         : 'True',
                'enable_gz_head_cam_color': 'True',
                'enable_gz_head_cam_depth': 'True',
                'enable_gz'               : 'True',
            }.items()
        ),
        # Launch Robot No. 2
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             os.path.join(get_package_share_directory('sobit_pro_bringup'),
        #             'launch',
        #             'robot.launch.py')
        #         ])
        #     ]),
        #     launch_arguments={
        #         'robot_name': robot_name if (robot_id+1) == 0 else robot_name + '_' + str(robot_id+1),
        #         'head_camera_name'        : head_camera_name,
        #         'enable_mb'               : 'True',
        #         'enable_arm'              : 'True',
        #         'enable_head'             : 'True',
        #         'robot_coords_x'          : '-5.5', # x 
        #         'robot_coords_y'          : '-2.5', # y
        #         'robot_coords_z'          : '0.01', # z
        #         'robot_coords_Y'          : '0.0',  # yaw
        #         'enable_gz_lidar'         : 'True',
        #         'enable_gz_head_cam_color': 'True',
        #         'enable_gz_head_cam_depth': 'True',
        #         'enable_gz'               : 'True',
        #     }.items()
        # ),
        rviz_node,
    ])
