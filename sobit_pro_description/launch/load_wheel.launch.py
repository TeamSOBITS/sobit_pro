import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    robot_namespace = "sobit_pro"

    wheel_description = os.path.join(get_package_share_directory(
        'sobit_pro_description'), 
        'urdf',
        'wheel',
        'wheel_2.urdf.xacro'
    )

    xacro_arguments = {
        'robot_namespace': robot_namespace
    }

    wheel_description_config = xacro.process_file(
        wheel_description,
        mappings=xacro_arguments
    )

    wheel_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="wheel_state_publisher",
        namespace=robot_namespace,
        parameters=[{
            "frame_prefix": robot_namespace + '/',
            "robot_description": wheel_description_config.toxml(),
            "use_sim_time": True,
        }],
        output="screen",
    )

    return LaunchDescription([
        wheel_state_publisher_node,
    ])
