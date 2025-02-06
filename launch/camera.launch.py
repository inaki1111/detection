import launch
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    # Correct way to get the path of detection package
    package_share = FindPackageShare("detection").find("detection")

    # Path to the RealSense launch file
    realsense_launch_path = FindPackageShare("realsense2_camera").find("realsense2_camera") + "/launch/rs_launch.py"

    # Path to the RViz2 config file
    rviz_config_path = package_share + "/config/detection.rviz2"

    return launch.LaunchDescription([
        # Include the RealSense launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_path),
            launch_arguments={
                "enable_rgbd": "true",
                "enable_sync": "true",
                "align_depth.enable": "true",
                "enable_color": "true",
                "enable_depth": "true",
            }.items(),
        ),

        # Launch detect_object node
        launch_ros.actions.Node(
            package="detection",
            executable="detect_object",
            name="detect_object"
        ),

        # Launch object_position node
        launch_ros.actions.Node(
            package="detection",
            executable="object_position",
            name="object_position"
        ),

        # Launch RViz2 with the custom config file
        launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_path],
            output="screen"
        )
    ])
