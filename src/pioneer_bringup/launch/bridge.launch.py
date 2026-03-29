from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory("pioneer_bringup")

    scan_bridge_file = os.path.join(
        bringup_dir,
        "config",
        "scan_bridge.yaml",
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
            static_lidar_tf,
            bridge_main,
            bridge_scan,
        ]
    )
