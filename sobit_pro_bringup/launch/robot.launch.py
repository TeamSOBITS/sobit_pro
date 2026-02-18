import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

import xacro


def generate_launch_description():
    arg_robot_name = DeclareLaunchArgument('robot_name', default_value='sobit_pro')
    arg_head_camera = DeclareLaunchArgument('head_camera_name', default_value='xtion')

    arg_robot_coords_x = DeclareLaunchArgument('robot_coords_x', default_value='0')
    arg_robot_coords_y = DeclareLaunchArgument('robot_coords_y', default_value='0')
    arg_robot_coords_z = DeclareLaunchArgument('robot_coords_z', default_value='0')
    arg_robot_coords_Y = DeclareLaunchArgument('robot_coords_Y', default_value='0')

    arg_enable_gz = DeclareLaunchArgument('enable_gz', default_value='False')

    arg_enable_mb = DeclareLaunchArgument('enable_mb', default_value='True')
    arg_enable_arm = DeclareLaunchArgument('enable_arm', default_value='True')
    arg_enable_head = DeclareLaunchArgument('enable_head', default_value='True')

    arg_enable_gz_lidar = DeclareLaunchArgument('enable_gz_lidar', default_value='True')
    arg_enable_gz_head_cam_color = DeclareLaunchArgument('enable_gz_head_cam_color', default_value='True')
    arg_enable_gz_head_cam_depth = DeclareLaunchArgument('enable_gz_head_cam_depth', default_value='True')

    arg_use_serial_urg = DeclareLaunchArgument('use_serial_urg', default_value='False')

    return LaunchDescription([
        arg_robot_name,
        arg_head_camera,
        arg_robot_coords_x,
        arg_robot_coords_y,
        arg_robot_coords_z,
        arg_robot_coords_Y,
        arg_enable_gz,
        arg_enable_mb,
        arg_enable_arm,
        arg_enable_head,
        arg_enable_gz_lidar,
        arg_enable_gz_head_cam_color,
        arg_enable_gz_head_cam_depth,
        arg_use_serial_urg,
        OpaqueFunction(function = launch_gz),
    ])


def launch_gz(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    head_camera_name = LaunchConfiguration('head_camera_name').perform(context)

    robot_coords_x = LaunchConfiguration('robot_coords_x').perform(context)
    robot_coords_y = LaunchConfiguration('robot_coords_y').perform(context)
    robot_coords_z = LaunchConfiguration('robot_coords_z').perform(context)
    robot_coords_Y = LaunchConfiguration('robot_coords_Y').perform(context)

    enable_gz = LaunchConfiguration('enable_gz').perform(context)

    enable_mb = LaunchConfiguration('enable_mb').perform(context)
    enable_arm = LaunchConfiguration('enable_arm').perform(context)
    enable_head = LaunchConfiguration('enable_head').perform(context)

    enable_gz_lidar = LaunchConfiguration('enable_gz_lidar').perform(context)
    enable_gz_head_cam_color = LaunchConfiguration('enable_gz_head_cam_color').perform(context)
    enable_gz_head_cam_depth = LaunchConfiguration('enable_gz_head_cam_depth').perform(context)

    use_serial_urg = LaunchConfiguration('use_serial_urg').perform(context)

    robot_description = os.path.join(get_package_share_directory(
        'sobit_pro_description'), 
        'robots',
        'sobit_pro_robot.urdf.xacro'
    )
    robot_description_config = xacro.process_file(
        robot_description,
        mappings={
            'enable_mb'   : enable_mb,
            'enable_arm'  : enable_arm,
            'enable_head' : enable_head,
            'enable_gz'   : enable_gz,
            'robot_name'  : robot_name,
            'head_camera_name': head_camera_name,
            'enable_gz_lidar': enable_gz_lidar,
            'enable_gz_head_cam_color': enable_gz_head_cam_color,
            'enable_gz_head_cam_depth': enable_gz_head_cam_depth,
        })


    if (use_serial_urg == 'False'):
        urg_config = os.path.join(get_package_share_directory("sobit_pro_bringup"), "config", "ethernet_urg_node_params.yaml")
    else:
        urg_config = os.path.join(get_package_share_directory("sobit_pro_bringup"), "config", "serial_urg_node_params.yaml")


    if enable_gz == 'False':
        controller_config = os.path.join(get_package_share_directory(
            'sobit_pro_control'),
            'config',
            'controllers.yaml'
        )
        ros2_control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=robot_name,
            parameters=[
                {"robot_description": robot_description_config.toxml()}, controller_config],
            output="screen",
        )
        urg_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('urg_node'),
                    'launch',
                    'urg.launch.py'
                ])
            ]),
            launch_arguments={
                "config_file" : urg_config,
                "use_namespace" : "true",
                "namespace" : robot_name,
            }.items()
        )

        if (head_camera_name == "xtion"):
            camera_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([os.path.join(
                        get_package_share_directory('sobit_pro_bringup'),
                        'launch',
                        'xtion.launch.py')
                    ])
                ]),
                launch_arguments={
                    'tf_prefix': robot_name,
                    'namespace': 'head_camera',
                }.items()
            )
        elif (head_camera_name == "azure_kinect"):
            camera_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([os.path.join(
                        get_package_share_directory('sobit_pro_bringup'),
                        'launch',
                        'azure_kinect.launch.py')
                    ])
                ]),
                launch_arguments={
                    'namespace': robot_name,
                }.items()
            )
        elif (head_camera_name == "femtobolt"): # TODO
            camera_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([os.path.join(
                        get_package_share_directory('sobit_pro_bringup'),
                        'launch',
                        'femtobolt.launch.py')
                    ])
                ]),
                launch_arguments={
                    'namespace': robot_name,
                }.items()
            )
        else:
            camera_node = None

        rviz_config = PathJoinSubstitution([
            FindPackageShare('sobit_pro_bringup'),
            'rviz',
            'real.rviz'
        ])

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster',
        namespace=robot_name,
        arguments=[
            'joint_state_broadcaster',
            '-c', 'controller_manager',
            ],
    )

    joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_trajectory_controller',
        namespace=robot_name,
        arguments=[
            'joint_trajectory_controller',
            '-c', 'controller_manager', '--activate'
            ],
    )

    steer_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='steer_joint_trajectory_controller',
        namespace=robot_name,
        arguments=[
            'steer_joint_trajectory_controller',
            '-c', 'controller_manager', '--activate'
            ],
    )

    velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='velocity_controller',
        namespace=robot_name,
        arguments=[
            'velocity_controller',
            '-c', 'controller_manager', '--activate'
            ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_name,
        parameters=[
            {"frame_prefix": robot_name + '/'},
            {"robot_description": robot_description_config.toxml()},
            {"use_sim_time": True if enable_gz == 'True' else False},
        ],
        output="screen",
    )

    if enable_gz == 'True':
        gz_spawn_entity_node = Node(
            package='ros_gz_sim',
            executable='create',
            namespace=robot_name,
            arguments=[
                '-topic', '/' + robot_name + '/robot_description',
                '-name', robot_name,
                '-x', robot_coords_x,
                '-y', robot_coords_y,
                '-z', robot_coords_z,
                '-Y', robot_coords_Y,
            ],
            output='screen',
        )

        gz_bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=robot_name,
            arguments=[
                        "/" + robot_name + "/joint_states" + "@sensor_msgs/msg/JointState" + "[gz.msgs.Model",
                        "/" + robot_name + "/head_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[gz.msgs.CameraInfo",
                        "/" + robot_name + "/head_camera/color" + "@sensor_msgs/msg/Image" + "[gz.msgs.Image",
                        "/" + robot_name + "/head_camera/depth" + "@sensor_msgs/msg/Image" + "[gz.msgs.Image",
                        "/" + robot_name + "/head_camera/depth/points" + "@sensor_msgs/msg/PointCloud2" + "[gz.msgs.PointCloudPacked",
                        # "/" + robot_name + "/hand_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[gz.msgs.CameraInfo",
                        # "/" + robot_name + "/hand_camera/color" + "@sensor_msgs/msg/Image" + "[gz.msgs.Image",
                        # "/" + robot_name + "/hand_camera/depth" + "@sensor_msgs/msg/Image" + "[gz.msgs.Image",
                        # "/" + robot_name + "/hand_camera/depth/points" + "@sensor_msgs/msg/PointCloud2" + "[gz.msgs.PointCloudPacked",
                        "/" + robot_name + "/scan" + "@sensor_msgs/msg/LaserScan" + "[gz.msgs.LaserScan",

                        "/" + robot_name + "/scan/points" + "@sensor_msgs/msg/PointCloud2" + "[gz.msgs.PointCloudPacked",
                        # "/" + robot_name + "/imu" + "@sensor_msgs/msg/Imu" + "[gz.msgs.IMU",
                    ],
            output='screen'
        )

        rviz_config = PathJoinSubstitution([
            FindPackageShare('sobit_pro_bringup'),
            'rviz',
            'gazebo.rviz'
        ])


    library_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sobit_pro_library'),
                'launch',
                'library_server.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': robot_name,
            'enable_gz': enable_gz,
        }.items(),
    )

    move_base_node = Node(
        package="sobit_pro_control",
        executable="sobit_pro_control_node",
        name="sobit_pro_control",
        namespace=robot_name,
        parameters=[
            {"use_sim_time": True if enable_gz == 'True' else False},
        ],
        output="screen",
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name=robot_name+'_rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    if enable_gz == 'False':
        return [
            ros2_control_node,
            joint_state_broadcaster,
            joint_trajectory_controller,
            steer_joint_trajectory_controller,
            velocity_controller,
            robot_state_publisher_node,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=move_base_node,
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=library_server_launch,
                )
            ),
            urg_node,
            camera_node,
            rviz_node,
        ]

    else:
        return [
            gz_spawn_entity_node,
            gz_bridge_node,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=gz_spawn_entity_node,
                    on_exit=joint_state_broadcaster,
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=joint_trajectory_controller,
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=steer_joint_trajectory_controller,
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=velocity_controller,
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=move_base_node,
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=library_server_launch,
                )
            ),
            robot_state_publisher_node,
            rviz_node,
        ]
