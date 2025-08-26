import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    namespace_param_name = "namespace"
    namespace = LaunchConfiguration(namespace_param_name)
    namespace_launch_arg = DeclareLaunchArgument(namespace_param_name)

    tf_prefix_param_name = "tf_prefix"
    tf_prefix = LaunchConfiguration(tf_prefix_param_name)
    tf_prefix_launch_arg = DeclareLaunchArgument(tf_prefix_param_name)

    tf_args = [
        # _depth_frame と _rgb_frame を含まない変換のみ残す
        ["--frame-id", [tf_prefix, "/", namespace, "_depth_frame"],
         "--child-frame-id", [tf_prefix, "/", namespace, "_depth_optical_frame"],
         "--roll", "-1.5707963267948966", "--yaw", "-1.5707963267948966"],
        ["--frame-id", [tf_prefix, "/", namespace, "_rgb_frame"],
         "--child-frame-id", [tf_prefix, "/", namespace, "_rgb_optical_frame"],
         "--roll", "-1.5707963267948966", "--yaw", "-1.5707963267948966"],
    ]

    # 上記は _depth_frame, _rgb_frame を含む変換のため、削除したい場合は以下のように変更：
    tf_args = [
        ["--frame-id", [tf_prefix, "/", namespace, "_link"],
         "--child-frame-id", [tf_prefix, "/", namespace, "_depth_optical_frame"],
         "--roll", "-1.5707963267948966", "--yaw", "-1.5707963267948966"],
        ["--frame-id", [tf_prefix, "/", namespace, "_link"],
         "--child-frame-id", [tf_prefix, "/", namespace, "_rgb_optical_frame"],
         "--roll", "-1.5707963267948966", "--yaw", "-1.5707963267948966"],
    ]

    tf_nodes = [Node(package='tf2_ros', executable='static_transform_publisher', output='screen', arguments=args) for args in tf_args]

    return launch.LaunchDescription([namespace_launch_arg, tf_prefix_launch_arg] + tf_nodes)