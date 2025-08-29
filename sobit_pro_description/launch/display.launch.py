import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    use_gui = LaunchConfiguration('use_gui', default='True')

    robot_name = "sobit_pro"
    head_camera_name = "xtion" ## "xtion" or "azure_kinect"

    rviz_config = os.path.join(get_package_share_directory(
        'sobit_pro_description'), "rviz", "display.rviz")
    
    robot_description = os.path.join(get_package_share_directory(
        'sobit_pro_description'), 
        'robots',
        'sobit_pro_robot.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        robot_description,
        mappings={
            'enable_gz' : 'True',
            'robot_name' : robot_name,
            'head_camera_name' : head_camera_name,
            'enable_mb': 'True',
            'enable_arm': 'True',
            'enable_head': 'True',
        }
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_name,
        parameters=[{
            "frame_prefix": robot_name + '/',
            "robot_description": robot_description_config.toxml(),
            "use_sim_time": True,
        }],
        output="screen",
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        namespace=robot_name,
        condition=UnlessCondition(use_gui)
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        namespace=robot_name,
        condition=IfCondition(use_gui)
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ])