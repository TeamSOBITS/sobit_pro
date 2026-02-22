import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    arg_robot_name = DeclareLaunchArgument('robot_name', default_value='sobit_pro')
    arg_enable_gz = DeclareLaunchArgument('enable_gz', default_value='False')
    arg_enable_joint_action_server = DeclareLaunchArgument('enable_joint_action_server', default_value='True')
    arg_enable_wheel_action_server = DeclareLaunchArgument('enable_wheel_action_server', default_value='True')
    arg_enable_arm = DeclareLaunchArgument('enable_arm', default_value='True')
    arg_enable_head = DeclareLaunchArgument('enable_head', default_value='True')
    arg_enable_hand = DeclareLaunchArgument('enable_hand', default_value='True')

    return LaunchDescription([
        arg_robot_name,
        arg_enable_gz,
        arg_enable_joint_action_server,
        arg_enable_wheel_action_server,
        arg_enable_arm,
        arg_enable_head,
        arg_enable_hand,
        OpaqueFunction(function = launch_gz),
    ])


def launch_gz(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    enable_gz = LaunchConfiguration('enable_gz').perform(context)
    enable_joint_action_server = LaunchConfiguration('enable_joint_action_server')
    enable_wheel_action_server = LaunchConfiguration('enable_wheel_action_server')
    enable_arm = LaunchConfiguration('enable_arm').perform(context)
    enable_head = LaunchConfiguration('enable_head').perform(context)
    enable_hand = LaunchConfiguration('enable_hand').perform(context)

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
            {"enable_arm": True if enable_arm == 'True' else False},
            {"enable_head": True if enable_head == 'True' else False},
            {"enable_hand": True if enable_hand == 'True' else False},
        ],
        output="screen",
        condition=IfCondition(enable_joint_action_server),
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
        condition=IfCondition(enable_wheel_action_server),
    )


    return [
        joint_action_server_node,
        wheel_action_server_node,
    ]
