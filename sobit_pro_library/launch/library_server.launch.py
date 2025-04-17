import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    robot_name = "sobit_pro"
    robot_id = 0

    pose_config = os.path.join(
        get_package_share_directory("sobit_pro_library"),
        "config",
        "pose_list.yaml",
    )

    joint_action_server_node = Node(
        package="sobit_pro_library",
        executable="joint_action_server",
        name="joint_action_server",
        namespace=robot_name if robot_id == 0 else f"{robot_name}_{robot_id}",
        parameters=[pose_config],
        output="screen",
    )

    wheel_action_server_node = Node(
        package="sobit_pro_library",
        executable="wheel_action_server",
        name="wheel_action_server",
        namespace=robot_name if robot_id == 0 else f"{robot_name}_{robot_id}",
        output="screen",
    )


    return LaunchDescription([
        joint_action_server_node,
        wheel_action_server_node,
    ])