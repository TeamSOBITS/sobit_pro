from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
<<<<<<< HEAD
from launch_ros.substitutions import FindPackageShare
=======
from launch.actions import TimerAction, ExecuteProcess

>>>>>>> 73b2a67 ([ADD] Orbbec femtobolt launch and minimal launch)


def generate_launch_description():
    robot_name = 'sobit_pro'
<<<<<<< HEAD
    head_camera_name = "xtion" # 'xtion' or 'azure_kinect' or 'femtobolt
    only_mobile_base_hardware = False # You use only mobile base to URG serial
=======
    robot_id = 0
    bringup_pkg = robot_name + '_bringup'
    head_camera_name = "orbbec_femtobolt" # 'xtion' or 'azure_kinect'  ## TODO : orbbec femt bolt??

    rviz_config = os.path.join(get_package_share_directory(bringup_pkg), 'rviz', 'real.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace=robot_name,
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )
    # Delay the goal until robot + tf + controllers are ready
    delayed_initial_pose = TimerAction(
        period=3.0,   # seconds
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'action', 'send_goal',
                    '/sobit_pro/move_to_pose',
                    'sobits_interfaces/action/MoveToPose',
                    "{pose_name: 'initial_pose', time_allowance: {sec: 6, nanosec: 0}}"
                ],
                output='screen'
            )
        ]
    )


    urg_config = os.path.join(get_package_share_directory(bringup_pkg), "config", "urg_node_params.yaml")

    if (head_camera_name == "xtion"):
        camera_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([
            PathJoinSubstitution([os.path.join(get_package_share_directory(bringup_pkg), 'launch', 'xtion.launch.py')])]),
            launch_arguments={'tf_prefix': robot_name if robot_id == 0 else robot_name + '_' + str(robot_id),'namespace': 'head_camera',}.items(),)
    elif (head_camera_name == "azure_kinect"):
        camera_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([
            PathJoinSubstitution([os.path.join(get_package_share_directory(bringup_pkg), 'launch', 'azure_kinect.launch.py')])]),
            launch_arguments={'namespace': robot_name if robot_id == 0 else robot_name + '_' + str(robot_id),}.items(),)
    
    elif (head_camera_name == "orbbec_femtobolt"):
        camera_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    os.path.join(get_package_share_directory(bringup_pkg), 'launch', 'orbbec_femtobolt.launch.py')
                ])
            ]),
            launch_arguments={
                'tf_prefix': robot_name if robot_id == 0 else robot_name + '_' + str(robot_id),
                'namespace': 'head_camera',
            }.items(),
        )

    else:
        camera_node = None
>>>>>>> 73b2a67 ([ADD] Orbbec femtobolt launch and minimal launch)

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
<<<<<<< HEAD
=======
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
                "namespace" : robot_name if robot_id == 0 else robot_name + '_' + str(robot_id),
            }.items(),
        ),
        rviz_node,
        delayed_initial_pose
>>>>>>> 73b2a67 ([ADD] Orbbec femtobolt launch and minimal launch)
    ])
