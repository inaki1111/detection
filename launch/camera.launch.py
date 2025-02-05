#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # camera resolution

    color_width = LaunchConfiguration('color_width', default='640')
    color_height = LaunchConfiguration('color_height', default='480')
    depth_width = LaunchConfiguration('depth_width', default='640')
    depth_height = LaunchConfiguration('depth_height', default='480')
    fps = LaunchConfiguration('fps', default='30')

    realsense_launch_file = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    return LaunchDescription([
        #launch arguments
        DeclareLaunchArgument('color_width', default_value='640', description='width RGB'),
        DeclareLaunchArgument('color_height', default_value='480', description='height RGB'),
        DeclareLaunchArgument('depth_width', default_value='640', description='width Depth'),
        DeclareLaunchArgument('depth_height', default_value='480', description='height Depth'),
        DeclareLaunchArgument('fps', default_value='30', description='fps'),

        # realsense camera node
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera_node',
            parameters=[{
                'color_width': int(color_width.perform(None)),
                'color_height': int(color_height.perform(None)),
                'depth_width': int(depth_width.perform(None)),
                'depth_height': int(depth_height.perform(None)),
                'fps': int(fps.perform(None))
            }]
        ),

        # fusiion node (rgb and depth)
        Node(
            package='detection',
            executable='fuse_image',
            name='fuse_image',
            output='screen'
        )
    ])

def get_package_share_directory(package_name):
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(package_name)
