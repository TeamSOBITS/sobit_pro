import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

import xacro

def generate_launch_description():
    arg_robot_name = DeclareLaunchArgument('robot_name', default_value='sobit_pro')
    arg_robot_coords_x = DeclareLaunchArgument('robot_coords_x', default_value='0')
    arg_robot_coords_y = DeclareLaunchArgument('robot_coords_y', default_value='0')
    arg_robot_coords_Y = DeclareLaunchArgument('robot_coords_Y', default_value='0')



    return LaunchDescription([
        arg_robot_name,
        arg_robot_coords_x,
        arg_robot_coords_y,
        arg_robot_coords_Y,
        OpaqueFunction(function = launch_gz),
    ])


def launch_gz(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    robot_coords_x = LaunchConfiguration('robot_coords_x').perform(context)
    robot_coords_y = LaunchConfiguration('robot_coords_y').perform(context)
    robot_coords_Y = LaunchConfiguration('robot_coords_Y').perform(context)


    robot_description = os.path.join(get_package_share_directory(
        'sobit_pro_description'), 
        'robots',
        'sobit_pro_robot.urdf.xacro'
    )
    robot_description_config = xacro.process_file(
        robot_description,
        mappings={
            'enable_gz' : 'False',
            'robot_name' : robot_name,
        })


    joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active',
             '--controller-manager', robot_name+'/controller_manager',
             'joint_state_broadcaster'
        ],
        output='screen'
    )

    velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'configured',
             '--controller-manager', robot_name+'/controller_manager',
             'velocity_controller'
        ],
        output='screen'
    )

    joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active',
             '--controller-manager', robot_name+'/controller_manager',
             'joint_trajectory_controller'
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
        ],
        output="screen",
    )

    controller_pkg = robot_name + "_control"
    controller_config = os.path.join(
        get_package_share_directory(
            controller_pkg), "config", "controllers.yaml"
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_config.toxml()}, controller_config],
        output="screen",
        namespace=robot_name,
    )

    return [
        ros2_control_node,
        joint_state_broadcaster,
        velocity_controller,
        joint_trajectory_controller,
        robot_state_publisher_node,
    ]