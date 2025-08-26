import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    arg_robot_name = DeclareLaunchArgument('robot_name', default_value='sobit_pro')
    arg_robot_id = DeclareLaunchArgument('robot_id', default_value='0')
    arg_enable_gz = DeclareLaunchArgument('enable_gz', default_value='True')

    return LaunchDescription([
        arg_robot_name,
        arg_robot_id,
        arg_enable_gz,
        OpaqueFunction(function=launch_gz),
    ])

def launch_gz(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    robot_id = int(LaunchConfiguration('robot_id').perform(context))
    enable_gz = LaunchConfiguration('enable_gz').perform(context)

    # Determine the namespace based on robot name and ID
    namespace = robot_name if robot_id == 0 else f"{robot_name}_{robot_id}"

    # Use simulation time if gz is enabled
    use_sim_time = True if enable_gz == 'True' else False


    pose_config = os.path.join(
        get_package_share_directory("sobit_pro_library"),
        "config",
        "pose_list.yaml",
    )

    joint_action_server_node = Node(
        package="sobit_pro_library",
        executable="joint_action_server",
        name="joint_action_server",
        namespace=namespace,
        parameters=[pose_config, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    wheel_action_server_node = Node(
        package="sobit_pro_library",
        executable="wheel_action_server",
        name="wheel_action_server",
        namespace=namespace,
        parameters=[pose_config, {"use_sim_time": use_sim_time}],
        output="screen",
    )


    return [joint_action_server_node, wheel_action_server_node]