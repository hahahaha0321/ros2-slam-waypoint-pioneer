from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory("pioneer_description"),
        "urdf",
        "pioneer.urdf",
    )

    world_file = os.path.join(
        get_package_share_directory("pioneer_gazebo"),
        "worlds",
        "james_oval.world",
    )

    with open(urdf_file, "r") as f:
        robot_desc = f.read()

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_desc},
            {"use_sim_time": True},
        ],
    )

    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        output="screen",
    )

    spawn_robot = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "ros_gz_sim",
            "create",
            "-file",
            urdf_file,
            "-name",
            "pioneer",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.2",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            joint_state_publisher,
            robot_state_publisher,
            gazebo,
            TimerAction(period=4.0, actions=[spawn_robot]),
        ]
    )
