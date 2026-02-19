import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    # -------------------------------------------------
    # Arguments
    # -------------------------------------------------
    namespace = LaunchConfiguration("namespace")
    tf_prefix = LaunchConfiguration("tf_prefix")

    args = [
        DeclareLaunchArgument("namespace", default_value="sobit_pro/head_camera"),
        DeclareLaunchArgument("tf_prefix", default_value="sobit_pro"),
    ]

    # -------------------------------------------------
    # Camera parameters
    # -------------------------------------------------
    camera_params = [
        {"enable_color": True},
        {"enable_depth": True},
        {"enable_point_cloud": True},
        {"enable_colored_point_cloud": False},
        {"depth_registration": True},
        {"ordered_pc": True},
        {"align_mode": "SW"},

        {"color_width": 1280},
        {"color_height": 720},
        {"color_fps": 30},

        {"depth_width": 640},
        {"depth_height": 576},
        {"depth_fps": 30},

        {"frame_id":       "sobit_pro/head_camera_base"},  # root of FemtoBolt camera
        {"color_frame_id": "sobit_pro/head_camera_rgb_optical_frame"},
        {"depth_frame_id": "sobit_pro/head_camera_depth_optical_frame"},
        {"cloud_frame_id": "sobit_pro/head_camera_depth_optical_frame"},

        {"publish_tf": False},
        {"enable_publish_extrinsic": False},
    ]

    # -------------------------------------------------
    # Camera node (component)
    # -------------------------------------------------
    camera_container = GroupAction([
        PushRosNamespace(namespace),
        ComposableNodeContainer(
            name="femtobolt_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="orbbec_camera",
                    plugin="orbbec_camera::OBCameraNodeDriver",
                    name="femtobolt_driver",
                    parameters=camera_params,
                    remappings=[
                        ("color/image_raw",
                         ["/", tf_prefix, "/head_camera/rgb/image_raw"]),
                        ("color/camera_info",
                         ["/", tf_prefix, "/head_camera/rgb/camera_info"]),

                        ("depth/image_raw",
                         ["/", tf_prefix, "/head_camera/depth_registered/image_raw"]),
                        ("depth/camera_info",
                         ["/", tf_prefix, "/head_camera/depth_registered/camera_info"]),

                        ("depth_registered/points",
                         ["/", tf_prefix, "/head_camera/depth_registered/points"]),
                        ("depth/points",
                         ["/", tf_prefix, "/head_camera/depth/points"]),
                    ]
                ),
            ],
            output="screen",
        )
    ])

    # -------------------------------------------------
    # STATIC TFs — CRITICAL
    # -------------------------------------------------

    static_tfs = [

        # Parent: sobit_pro/head_tilt_link (real robot)
        # Child: sobit_pro/head_camera_base (FemtoBolt root frame)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "0", "0", "0.02",    # translation (tune later)
                "0", "0", "0",       # rotation
                "sobit_pro/head_tilt_link",
                "sobit_pro/head_camera_base",
            ],
        ),

        # Camera base → RGB optical frame
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "0", "0", "0",
                "-1.5708", "0", "-1.5708",
                "sobit_pro/head_camera_base",
                "sobit_pro/head_camera_rgb_optical_frame",
            ],
        ),

        # Camera base → Depth optical frame
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "0", "0", "0",
                "-1.5708", "0", "-1.5708",
                "sobit_pro/head_camera_base",
                "sobit_pro/head_camera_depth_optical_frame",
            ],
        ),
    ]

    return LaunchDescription(args + [camera_container] + static_tfs)
