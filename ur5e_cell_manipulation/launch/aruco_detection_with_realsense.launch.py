from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    realsence_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
        launch_arguments={"device_type": "d435", "publish_tf": "false"}.items(),
    )

    realsence_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "-0.00640796",
            "-0.0551158",
            "0.00796752",
            "-0.529877",
            "0.53423",
            "-0.467606",
            "-0.463867",
            "tool0",
            "camera_link",
        ],
    )

    aruco_detection_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("aruco_ros"), "launch", "marker_publisher.launch.py"]
            )
        ),
        launch_arguments={
            "camera_frame": "camera_color_optical_frame",
            "reference_frame": "world",
            "marker_size_arg": "0.02",
        }.items(),
    )

    ld = LaunchDescription(
        [realsence_launch, realsence_tf_node, aruco_detection_launch]
    )

    return ld
