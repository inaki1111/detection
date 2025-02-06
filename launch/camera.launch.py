import os
import launch
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument

def generate_launch_description():
    # Obtener la ruta del paquete detection
    package_share = FindPackageShare(package="detection").find("detection")
    realsense_share = FindPackageShare(package="realsense2_camera").find("realsense2_camera")

    # Obtener rutas de archivos necesarios
    realsense_launch_path = os.path.join(realsense_share, "launch", "rs_launch.py")
    rviz_config_path = os.path.join(package_share, "config", "detection.rviz2")

    return launch.LaunchDescription([
        # Incluir el launch file de RealSense
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_path),
            launch_arguments=[
                ("enable_rgbd", "true"),
                ("enable_sync", "true"),
                ("align_depth.enable", "true"),
                ("enable_color", "true"),
                ("enable_depth", "true"),
            ]
        ),

        # Nodo para detección de objetos
        launch_ros.actions.Node(
            package="detection",
            executable="detect_object",
            name="detect_object",
            output="screen"
        ),

        # Nodo para calcular posición de objetos
        launch_ros.actions.Node(
            package="detection",
            executable="object_position",
            name="object_position",
            output="screen"
        ),

        # Nodo para lanzar RViz con configuración personalizada
        launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_path],
            output="screen"
        )
    ])
