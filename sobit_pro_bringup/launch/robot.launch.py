import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

import xacro


def generate_launch_description():
    arg_robot_name = DeclareLaunchArgument('robot_name', default_value='sobit_pro')

    arg_robot_coords_x = DeclareLaunchArgument('robot_coords_x', default_value='0')
    arg_robot_coords_y = DeclareLaunchArgument('robot_coords_y', default_value='0')
    arg_robot_coords_z = DeclareLaunchArgument('robot_coords_z', default_value='0')
    arg_robot_coords_Y = DeclareLaunchArgument('robot_coords_Y', default_value='0')

    arg_enable_gz = DeclareLaunchArgument('enable_gz', default_value='False')

    return LaunchDescription([
        arg_robot_name,
        arg_robot_coords_x,
        arg_robot_coords_y,
        arg_robot_coords_z,
        arg_robot_coords_Y,
        arg_enable_gz,
        OpaqueFunction(function = launch_gz),
    ])


def launch_gz(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)

    robot_coords_x = LaunchConfiguration('robot_coords_x').perform(context)
    robot_coords_y = LaunchConfiguration('robot_coords_y').perform(context)
    robot_coords_z = LaunchConfiguration('robot_coords_z').perform(context)
    robot_coords_Y = LaunchConfiguration('robot_coords_Y').perform(context)

    enable_gz = LaunchConfiguration('enable_gz').perform(context)

    robot_description = os.path.join(get_package_share_directory(
        'sobit_pro_description'), 
        'robots',
        'sobit_pro_robot.urdf.xacro'
    )
    robot_description_config = xacro.process_file(
        robot_description,
        mappings={
            'enable_gz' : enable_gz,
            'robot_name' : robot_name,
        })
    
    rviz_config = PathJoinSubstitution([
        FindPackageShare('sobit_pro_bringup'),
        'rviz',
        'gazebo.rviz'
    ]) if enable_gz == 'True' else PathJoinSubstitution([
        FindPackageShare('sobit_pro_bringup'),
        # FindPackageShare('robocup_opl_cml'),
        'rviz',
        'real.rviz'
    ])

    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     namespace=robot_name,
    #     arguments=['-d', rviz_config],
    #     output='screen',
    # )

    if enable_gz == 'False':
        controller_config = os.path.join(get_package_share_directory(
            'sobit_pro_control'),
            'config',
            'controllers.yaml'
        )

        controller_manager = Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=robot_name,
            parameters=[
                {"robot_description": robot_description_config.toxml()}, controller_config],
            output="screen",
        )

    joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
            '--set-state', 'active',
            '--controller-manager', robot_name+'/controller_manager',
            # '--use-sim-time',
            'joint_state_broadcaster'
        ],
        output='screen'
    )

    joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
            '--set-state', 'active',
            '--controller-manager', robot_name+'/controller_manager',
            # '--use-sim-time',
            'joint_trajectory_controller'
        ],
        output='screen'
    )

    steer_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
            '--set-state', 'active',
            '--controller-manager', robot_name+'/controller_manager',
            # '--use-sim-time',
            'steer_joint_trajectory_controller'
        ],
        output='screen'
    )

    velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
            '--set-state', 'active',
            '--controller-manager', robot_name+'/controller_manager',
            # '--use-sim-time',
            'velocity_controller'
        ],
        output='screen'
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
                        "/" + robot_name + "/joint_states" + "@sensor_msgs/msg/JointState" + "[ignition.msgs.Model",
                        # "/model/" + robot_name + "/pose" + "@geometry_msgs/msg/Pose" + "[ignition.msgs.Pose",
                        # "/" + robot_name + "/base_front_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[ignition.msgs.CameraInfo",
                        # "/" + robot_name + "/base_front_camera/color" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                        # "/" + robot_name + "/base_front_camera/depth" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                        # "/" + robot_name + "/base_back_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[ignition.msgs.CameraInfo",
                        # "/" + robot_name + "/base_back_camera/color" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                        # "/" + robot_name + "/base_back_camera/depth" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                        # "/" + robot_name + "/head_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[ignition.msgs.CameraInfo",
                        # "/" + robot_name + "/head_camera/color" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                        # "/" + robot_name + "/head_camera/depth" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                        # "/" + robot_name + "/head_camera/depth/points" + "@sensor_msgs/msg/PointCloud2" + "[ignition.msgs.PointCloudPacked",
                        # "/" + robot_name + "/hand_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[ignition.msgs.CameraInfo",
                        # "/" + robot_name + "/hand_camera/color" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                        # "/" + robot_name + "/hand_camera/depth" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                        # "/" + robot_name + "/hand_camera/depth/points" + "@sensor_msgs/msg/PointCloud2" + "[ignition.msgs.PointCloudPacked",
                        # "/" + robot_name + "/lidar/scan" + "@sensor_msgs/msg/LaserScan" + "[ignition.msgs.LaserScan",
                        # "/" + robot_name + "/lidar/scan/points" + "@sensor_msgs/msg/PointCloud2" + "[ignition.msgs.PointCloudPacked",
                        # "/" + robot_name + "/imu" + "@sensor_msgs/msg/Imu" + "[ignition.msgs.IMU",
                    ],
            output='screen'
        )

    if enable_gz == 'False':
        return [
            controller_manager,
            joint_state_broadcaster,
            joint_trajectory_controller,
            steer_joint_trajectory_controller,
            velocity_controller,
            robot_state_publisher_node,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[move_base_node],
                )
            ),
            # rviz_node,
        ]

    else:
        return [
            gz_spawn_entity_node,
            gz_bridge_node,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=gz_spawn_entity_node,
                    on_exit=[joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[joint_trajectory_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[steer_joint_trajectory_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[velocity_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[move_base_node],
                )
            ),
            robot_state_publisher_node,
            # rviz_node,
        ]
