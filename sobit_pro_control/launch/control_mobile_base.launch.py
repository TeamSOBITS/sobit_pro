from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    arg_robot_name = DeclareLaunchArgument('robot_name', default_value='sobit_pro')
    arg_enable_gz  = DeclareLaunchArgument('enable_gz', default_value='False')

    return LaunchDescription([
        arg_robot_name,
        arg_enable_gz,
        OpaqueFunction(function = launch_gz),
    ])


def launch_gz(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    enable_gz  = LaunchConfiguration('enable_gz').perform(context)
    
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

    return LaunchDescription([
        move_base_node,
    ])
