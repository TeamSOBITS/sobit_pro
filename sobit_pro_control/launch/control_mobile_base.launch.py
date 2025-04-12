from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    robot_name = "sobit_pro"
    robot_id = 0
    enable_gz = 'True'
    
    move_base_node = Node(
        package="sobit_pro_control",
        executable="sobit_pro_control_node",
        name="sobit_pro_control",
        namespace=robot_name if robot_id == 0 else f"{robot_name}_{robot_id}",
        parameters=[
            {"use_sim_time": True if enable_gz == 'True' else False},
        ],
        output="screen",
    )

    return LaunchDescription([
        move_base_node,
    ])
