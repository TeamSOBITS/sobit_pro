import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    arg_robot_name = DeclareLaunchArgument('robot_name', default_value='sobit_pro')
    arg_enable_gz = DeclareLaunchArgument('enable_gz', default_value='False')

    return LaunchDescription([
        arg_robot_name,
        arg_enable_gz,
        OpaqueFunction(function = launch_gz),
    ])


def launch_gz(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    enable_gz = LaunchConfiguration('enable_gz').perform(context)

    pose_config = os.path.join(
        get_package_share_directory("sobit_pro_library"),
        "config",
        "pose_list.yaml",
    )

    joint_action_server_node = Node(
        package="sobit_pro_library",
        executable="joint_action_server",
        name="joint_action_server",
        namespace=robot_name,
        parameters=[pose_config,
            {"use_sim_time": True if enable_gz == 'True' else False},
        ],
        output="screen",
    )

    wheel_action_server_node = Node(
        package="sobit_pro_library",
        executable="wheel_action_server",
        name="wheel_action_server",
        namespace=robot_name,
        parameters=[
            {"use_sim_time": True if enable_gz == 'True' else False},
        ],
        output="screen",
    )


    return [
        joint_action_server_node,
        wheel_action_server_node,
    ]