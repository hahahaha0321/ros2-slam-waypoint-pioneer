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

    bringup_dir = get_package_share_directory("pioneer_bringup")
    scan_bridge_file = os.path.join(
        bringup_dir,
        "config",
        "scan_bridge.yaml",
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

    static_lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_frame_alias",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "lidar_link",
            "pioneer/base_link/lidar",
        ],
        output="screen",
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

    bridge_main = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "ros_gz_bridge",
            "parameter_bridge",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    bridge_scan = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "ros_gz_bridge",
            "parameter_bridge",
            "--ros-args",
            "-p",
            f"config_file:={scan_bridge_file}",
            "-p",
            "use_sim_time:=true",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            joint_state_publisher,
            robot_state_publisher,
            static_lidar_tf,
            gazebo,
            TimerAction(period=4.0, actions=[spawn_robot]),
            TimerAction(period=6.0, actions=[bridge_main]),
            TimerAction(period=7.0, actions=[bridge_scan]),
        ]
    )
