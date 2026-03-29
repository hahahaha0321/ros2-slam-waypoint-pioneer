from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory("pioneer_bringup")

    slam_params_file = os.path.join(
        bringup_dir,
        "config",
        "slam_params.yaml",
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("slam_toolbox"),
                    "launch",
                    "online_async_launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "slam_params_file": slam_params_file,
        }.items(),
    )

    return LaunchDescription([slam_launch])
