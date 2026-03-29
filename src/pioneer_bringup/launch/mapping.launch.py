from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory("pioneer_bringup")

    rviz_config_file = os.path.join(
        bringup_dir,
        "config",
        "mapping.rviz",
    )

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "bringup.launch.py")
        )
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "slam.launch.py")
        )
    )

    rviz_launch = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
    )

    teleop_launch = ExecuteProcess(
        cmd=[
            "gnome-terminal",
            "--",
            "bash",
            "-c",
            "source ~/ros_ws/install/setup.bash && "
            "ros2 run teleop_twist_keyboard teleop_twist_keyboard "
            "--ros-args -p stamped:=false",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            bringup_launch,
            TimerAction(period=5.0, actions=[slam_launch]),
            TimerAction(period=7.0, actions=[rviz_launch]),
            TimerAction(period=9.0, actions=[teleop_launch]),
        ]
    )
