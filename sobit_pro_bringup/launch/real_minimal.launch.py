from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name = 'sobit_pro'
    # keep xtion default for compatibility; switch manually when needed
    head_camera_name = 'xtion'  # 'xtion' or 'azure_kinect' or 'orbbec_femtobolt'
    only_mobile_base_hardware = False  # use serial URG only for mobile-base-only setup
    delayed_initial_pose = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'action', 'send_goal',
                    f'/{robot_name}/move_to_pose',
                    'sobits_interfaces/action/MoveToPose',
                    "{pose_name: 'initial_pose', time_allowance: {sec: 6, nanosec: 0}}",
                ],
                output='screen',
            )
        ],
    )

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
                'enable_mb': 'True',
                'enable_arm': 'True',
                'enable_head': 'True',
                'enable_gz': 'False',
                'use_serial_urg': 'True' if only_mobile_base_hardware else 'False',
            }.items()
        ),
        delayed_initial_pose,
    ])
