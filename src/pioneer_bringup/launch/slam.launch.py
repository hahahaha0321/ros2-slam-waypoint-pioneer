from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    slam_params_file = os.path.join(
        get_package_share_directory("pioneer_bringup"),
        "config",
        "slam_params.yaml",
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params_file,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([slam_toolbox])
