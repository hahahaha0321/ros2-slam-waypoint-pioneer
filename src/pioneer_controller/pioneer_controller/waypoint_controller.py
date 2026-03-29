#!/usr/bin/env python3

import math
import csv
from pathlib import Path

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import xml.etree.ElementTree as ET


def wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min(value, max_value), min_value)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def point_in_polygon(x: float, y: float, polygon) -> bool:
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]

        if (y1 > y) != (y2 > y):
            x_intersect = (x2 - x1) * (y - y1) / ((y2 - y1) + 1e-12) + x1
            if x <= x_intersect:
                inside = not inside

    return inside


def point_on_segment(
    px: float, py: float, ax: float, ay: float, bx: float, by: float, tol: float = 1e-9
) -> bool:
    abx = bx - ax
    aby = by - ay
    apx = px - ax
    apy = py - ay

    cross = abs(abx * apy - aby * apx)
    if cross > tol:
        return False

    dot = apx * abx + apy * aby
    if dot < -tol:
        return False

    sq_len = abx * abx + aby * aby
    if dot - sq_len > tol:
        return False

    return True


def point_on_polygon_boundary(x: float, y: float, polygon, tol: float = 1e-9) -> bool:
    for i in range(len(polygon)):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % len(polygon)]
        if point_on_segment(x, y, x1, y1, x2, y2, tol):
            return True
    return False


def rotate_2d(x: float, y: float, yaw: float):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * x - s * y, s * x + c * y


def normalize_pose_values(values: str):
    nums = [float(v) for v in values.split()]
    while len(nums) < 6:
        nums.append(0.0)
    return nums[:6]


def compose_2d_pose(parent_pose, local_pose):
    px, py, _, _, _, pyaw = parent_pose
    lx, ly, _, _, _, lyaw = local_pose
    rx, ry = rotate_2d(lx, ly, pyaw)
    return (px + rx, py + ry, 0.0, 0.0, 0.0, wrap(pyaw + lyaw))


class BugWaypointController(Node):
    def __init__(self):
        super().__init__("bug_waypoint_controller")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.scan = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.got_odom = False

        self.waypoint_file = Path.home() / "ros_ws/waypoints.txt"
        self.world_file = (
            Path.home() / "ros_ws/src/pioneer_gazebo/worlds/james_oval.world"
        )
        self.oval_polygon, self.world_obstacles = self.load_world_geometry(
            self.world_file
        )

        self.waypoints = self.load_waypoints(self.waypoint_file)
        if len(self.waypoints) == 0:
            self.get_logger().error("No waypoints loaded! Check waypoints.txt")

        self.current_waypoint_index = 0

        self.mode = "GO_TO_GOAL"

        self.pos_tol = 0.12
        self.yaw_tol = 0.10
        self.max_lin = 0.18
        self.max_ang = 0.90
        self.k_lin = 0.55
        self.k_ang_heading = 1.8
        self.k_ang_yaw = 1.4
        self.rotate_in_place_threshold = 0.30

        self.desired_wall_distance = 0.68
        self.kp_wall_dist = 0.85
        self.kp_wall_parallel = 0.30
        self.wall_follow_speed = 0.10

        self.front_block_dist = 0.80
        self.front_emergency_dist = 0.42
        self.wall_target = 0.52
        self.wall_target_band = 0.12
        self.too_close_right = 0.34
        self.side_open_dist = 0.95
        self.leave_commit_dist = 0.30
        self.hit_tol = 0.22
        self.step_margin = 0.03

        self.start_time_sec = self.get_clock().now().nanoseconds / 1e9

        self.wall_speed = 0.09
        self.wall_turn_limit = 0.70
        self.wall_k_dist = 1.1
        self.wall_k_parallel = 0.7

        self.align_timeout = 1.2
        self.align_start_time_sec = 0.0
        self.align_front_clear = 0.55
        self.align_parallel_tol = 0.30
        self.align_max_turn = 0.45

        self.hit_point = None
        self.left_hit_point = False
        self.minimum_distance_to_goal = float("inf")
        self.hit_distance_to_goal = float("inf")
        self.wall_follow_start_time = 0.0
        self.best_wall_distance_to_goal = float("inf")

        self.path_log = []
        self.last_log_time = self.get_clock().now().nanoseconds / 1e9
        self.log_period = 0.25
        self.output_csv = Path.home() / "bug_path_log.csv"

        self.get_logger().info("Bug waypoint controller started")

    def load_world_geometry(self, filepath: Path):
        try:
            root = ET.parse(filepath).getroot()
        except Exception as e:
            raise RuntimeError(f"Failed to load world file {filepath}: {e}")

        world = root.find("world")
        if world is None:
            raise RuntimeError(f"No <world> tag found in {filepath}")

        markers = {}
        obstacles = []

        for model in world.findall("model"):
            model_name = model.get("name", "")
            model_pose_text = model.findtext("pose", default="0 0 0 0 0 0")
            model_pose = normalize_pose_values(model_pose_text)

            if model_name.startswith("marker"):
                marker_idx = int(model_name.replace("marker", ""))
                markers[marker_idx] = (model_pose[0], model_pose[1])

            for link in model.findall("link"):
                link_pose_text = link.findtext("pose", default="0 0 0 0 0 0")
                link_pose = compose_2d_pose(
                    model_pose, normalize_pose_values(link_pose_text)
                )

                for collision in link.findall("collision"):
                    col_pose_text = collision.findtext("pose", default="0 0 0 0 0 0")
                    col_pose = compose_2d_pose(
                        link_pose, normalize_pose_values(col_pose_text)
                    )

                    cyl = collision.find("geometry/cylinder")
                    box = collision.find("geometry/box")

                    if cyl is not None:
                        radius = float(cyl.findtext("radius"))
                        obstacles.append(
                            {
                                "type": "circle",
                                "name": f"{model_name}/{collision.get('name', 'collision')}",
                                "x": col_pose[0],
                                "y": col_pose[1],
                                "radius": radius,
                            }
                        )

                    elif box is not None:
                        sx, sy, *_ = [float(v) for v in box.findtext("size").split()]
                        obstacles.append(
                            {
                                "type": "box",
                                "name": f"{model_name}/{collision.get('name', 'collision')}",
                                "x": col_pose[0],
                                "y": col_pose[1],
                                "yaw": col_pose[5],
                                "sx": sx,
                                "sy": sy,
                            }
                        )

        required_markers = [1, 2, 3, 4, 5, 6, 7, 8]
        missing = [m for m in required_markers if m not in markers]
        if missing:
            raise RuntimeError(f"Missing marker(s) in world file: {missing}")

        oval_polygon = [markers[i] for i in required_markers]

        self.get_logger().info(
            f"Loaded world geometry from {filepath}: "
            f"{len(oval_polygon)} oval marker points, {len(obstacles)} obstacle footprint(s)"
        )

        return oval_polygon, obstacles

    def waypoint_hits_obstacle(self, x: float, y: float):
        for obs in self.world_obstacles:
            if obs["type"] == "circle":
                if math.hypot(x - obs["x"], y - obs["y"]) <= obs["radius"]:
                    return obs["name"]

            elif obs["type"] == "box":
                dx = x - obs["x"]
                dy = y - obs["y"]
                lx, ly = rotate_2d(dx, dy, -obs["yaw"])
                if abs(lx) <= obs["sx"] / 2.0 and abs(ly) <= obs["sy"] / 2.0:
                    return obs["name"]

        return None

    def validate_waypoint(self, line_num: int, x: float, y: float):
        if point_on_polygon_boundary(x, y, self.oval_polygon) or not point_in_polygon(
            x, y, self.oval_polygon
        ):
            raise ValueError(
                f"Line {line_num}: waypoint ({x:.3f}, {y:.3f}) is outside the oval or on its boundary"
            )

        hit_name = self.waypoint_hits_obstacle(x, y)
        if hit_name is not None:
            raise ValueError(
                f"Line {line_num}: waypoint ({x:.3f}, {y:.3f}) lies on obstacle '{hit_name}'"
            )

    def scan_callback(self, msg: LaserScan):
        self.scan = msg

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.got_odom = True

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def publish_cmd(self, v: float, w: float):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

    def get_time_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def log_path_sample(self):
        now = self.get_time_sec()
        if now - self.last_log_time >= self.log_period:
            self.path_log.append((now, self.x, self.y, self.yaw, self.mode))
            self.last_log_time = now

    def save_path_log(self):
        try:
            with open(self.output_csv, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["time_s", "x_m", "y_m", "yaw_rad", "mode"])
                writer.writerows(self.path_log)
            self.get_logger().info(f"Saved path log to {self.output_csv}")
        except Exception as e:
            self.get_logger().error(f"Could not save path log: {e}")

    def load_waypoints(self, filepath: Path):
        waypoints = []

        try:
            with open(filepath, "r") as f:
                for line_num, line in enumerate(f, start=1):
                    line = line.strip()

                    if not line or line.startswith("#"):
                        continue

                    parts = [p.strip() for p in line.replace(",", " ").split()]

                    if len(parts) == 2:
                        x = float(parts[0])
                        y = float(parts[1])
                        yaw = 0.0
                    elif len(parts) == 3:
                        x = float(parts[0])
                        y = float(parts[1])
                        yaw = float(parts[2])
                    else:
                        raise ValueError(
                            f"Line {line_num}: expected 'x,y' or 'x,y,yaw'"
                        )

                    self.validate_waypoint(line_num, x, y)
                    waypoints.append((x, y, yaw))

        except Exception as e:
            self.get_logger().error(f"Failed to load waypoint file {filepath}: {e}")
            raise

        self.get_logger().info(
            f"Loaded {len(waypoints)} valid waypoint(s) from {filepath}"
        )
        return waypoints

    def get_range_at_angle(self, target_angle: float, window: int = 2) -> float:
        if self.scan is None or len(self.scan.ranges) == 0:
            return float("inf")

        ranges = self.scan.ranges
        best = float("inf")

        center_index = int(
            round((target_angle - self.scan.angle_min) / self.scan.angle_increment)
        )

        for k in range(-window, window + 1):
            i = center_index + k
            if 0 <= i < len(ranges):
                r = ranges[i]
                if math.isfinite(r) and r > 0.0:
                    best = min(best, r)

        return best

    def get_sector_distance(self, angle_min: float, angle_max: float) -> float:
        if self.scan is None:
            return float("inf")

        best = float("inf")
        angle = self.scan.angle_min

        for r in self.scan.ranges:
            if angle_min <= angle <= angle_max and math.isfinite(r) and r > 0.0:
                best = min(best, r)
            angle += self.scan.angle_increment

        return best

    def get_front_distance(self) -> float:
        return min(
            self.get_sector_distance(-0.35, 0.35),
            self.get_range_at_angle(0.0, window=4),
        )

    def get_front_left_distance(self) -> float:
        return self.get_range_at_angle(0.75, window=4)

    def get_left_distance(self) -> float:
        return self.get_range_at_angle(1.57, window=5)

    def get_front_right_distance(self) -> float:
        return self.get_range_at_angle(-0.75, window=4)

    def get_right_distance(self) -> float:
        return self.get_range_at_angle(-1.57, window=5)

    def get_back_right_distance(self) -> float:
        return self.get_range_at_angle(-2.35, window=4)

    def get_goal(self):
        return self.waypoints[self.current_waypoint_index]

    def distance_to_goal(self) -> float:
        goal_x, goal_y, _ = self.get_goal()
        return math.hypot(goal_x - self.x, goal_y - self.y)

    def angle_to_goal(self) -> float:
        goal_x, goal_y, _ = self.get_goal()
        return math.atan2(goal_y - self.y, goal_x - self.x)

    def heading_error_to_goal(self) -> float:
        return wrap(self.angle_to_goal() - self.yaw)

    def goal_direction_clearance(self) -> float:
        if self.scan is None:
            return float("inf")

        rel = self.heading_error_to_goal()
        return self.get_range_at_angle(rel, window=4)

    def obstacle_detected(self) -> bool:
        front = self.get_front_distance()
        front_left = self.get_front_left_distance()
        front_right = self.get_front_right_distance()

        self.get_logger().info(
            f"front={front:.2f}, fl={front_left:.2f}, fr={front_right:.2f}"
        )

        return (
            front < 0.65
            or (front < 0.85 and front_left < 0.75)
            or (front < 0.85 and front_right < 0.75)
        )

    def obstacle_too_close(self) -> bool:
        return self.get_front_distance() < self.front_emergency_dist

    def point_close(
        self, ax: float, ay: float, bx: float, by: float, tol: float
    ) -> bool:
        return math.hypot(ax - bx, ay - by) <= tol

    def enter_wall_follow(self):
        self.mode = "ALIGN_RIGHT_WALL"
        self.align_start_time_sec = self.get_time_sec()
        self.hit_point = (self.x, self.y)
        self.left_hit_point = False
        self.minimum_distance_to_goal = self.distance_to_goal()
        self.hit_distance_to_goal = self.minimum_distance_to_goal
        self.minimum_distance_to_goal = self.distance_to_goal()
        self.hit_distance_to_goal = self.minimum_distance_to_goal
        self.wall_follow_start_time = self.get_time_sec()
        self.best_wall_distance_to_goal = self.minimum_distance_to_goal
        self.get_logger().info(
            f"HIT at ({self.x:.2f}, {self.y:.2f}), min_d={self.minimum_distance_to_goal:.2f}"
        )

    def go_to_goal_step(self):
        distance = self.distance_to_goal()
        goal_yaw = self.get_goal()[2]
        heading_error = self.heading_error_to_goal()
        yaw_error = wrap(goal_yaw - self.yaw)
        now = self.get_clock().now().nanoseconds / 1e9

        if now - self.start_time_sec >= 1.5:
            if self.obstacle_too_close() or self.obstacle_detected():
                self.enter_wall_follow()
                return

        if distance > self.pos_tol:
            if abs(heading_error) > self.rotate_in_place_threshold:
                self.publish_cmd(
                    0.0,
                    clamp(
                        self.k_ang_heading * heading_error, -self.max_ang, self.max_ang
                    ),
                )
            else:
                v = clamp(self.k_lin * distance, 0.0, self.max_lin)
                w = clamp(
                    self.k_ang_heading * heading_error, -self.max_ang, self.max_ang
                )
                self.publish_cmd(v, w)
            return

        if abs(yaw_error) > self.yaw_tol:
            self.mode = "ROTATE_TO_FINAL_YAW"
            return

        self.stop_robot()
        self.get_logger().info(f"Waypoint {self.current_waypoint_index + 1} reached")
        self.current_waypoint_index += 1

        if self.current_waypoint_index >= len(self.waypoints):
            self.mode = "DONE"
        else:
            self.mode = "GO_TO_GOAL"

    def align_obstacle_on_right_step(self):
        front = self.get_front_distance()
        right = self.get_right_distance()
        front_right = self.get_front_right_distance()
        back_right = self.get_back_right_distance()
        elapsed = self.get_time_sec() - self.align_start_time_sec

        right_valid = math.isfinite(right) and right < 2.0
        fr_valid = math.isfinite(front_right) and front_right < 2.0
        br_valid = math.isfinite(back_right) and back_right < 2.0

        if fr_valid and br_valid:
            parallel_error = back_right - front_right
        else:
            parallel_error = 0.0

        self.get_logger().info(
            f"[ALIGN] front={front:.2f}, right={right:.2f}, fr={front_right:.2f}, br={back_right:.2f}, perr={parallel_error:.2f}"
        )

        if elapsed > self.align_timeout:
            self.mode = "FOLLOW_RIGHT_BOUNDARY"
            self.get_logger().info("ALIGN timeout -> FOLLOW_RIGHT_BOUNDARY")
            return

        if front > self.align_front_clear and (right_valid or fr_valid):
            self.mode = "FOLLOW_RIGHT_BOUNDARY"
            self.get_logger().info("ALIGN front clear -> FOLLOW_RIGHT_BOUNDARY")
            return

        if front < self.align_front_clear:
            self.publish_cmd(0.0, 0.35)
            return

        w = clamp(0.8 * parallel_error, -self.align_max_turn, self.align_max_turn)
        self.publish_cmd(0.04, w)

    def should_leave_wall(self) -> bool:
        if self.hit_point is None:
            return False

        distance = self.distance_to_goal()
        heading_clear = self.goal_direction_clearance()
        heading_error = abs(self.heading_error_to_goal())
        dist_from_hit = math.hypot(
            self.x - self.hit_point[0], self.y - self.hit_point[1]
        )

        if distance < self.minimum_distance_to_goal:
            self.minimum_distance_to_goal = distance

        if dist_from_hit < 0.2:
            return False

        progress_good = distance < (self.hit_distance_to_goal - 0.04)

        heading_good = heading_error < 0.75

        clear_good = heading_clear > max(0.50, distance + 0.02)

        if progress_good and heading_good and clear_good:
            self.get_logger().info(
                f"LEAVE wall: d={distance:.2f}, clear={heading_clear:.2f}, "
                f"hit_d={self.hit_distance_to_goal:.2f}"
            )
            return True

        return False

    def follow_right_boundary_step(self):
        if self.hit_point is not None:
            dist_from_hit = math.hypot(
                self.x - self.hit_point[0], self.y - self.hit_point[1]
            )

            if dist_from_hit > self.hit_tol:
                self.left_hit_point = True

            if self.left_hit_point and self.point_close(
                self.x, self.y, self.hit_point[0], self.hit_point[1], self.hit_tol
            ):
                self.stop_robot()
                self.mode = "FAILED"
                self.get_logger().warn(
                    "Returned to hit point, goal considered unreachable"
                )
                return

        if self.should_leave_wall():
            self.mode = "GO_TO_GOAL"
            return

        current_goal_distance = self.distance_to_goal()
        wall_follow_elapsed = self.get_time_sec() - self.wall_follow_start_time

        if current_goal_distance < self.best_wall_distance_to_goal:
            self.best_wall_distance_to_goal = current_goal_distance

        if wall_follow_elapsed > 4.0:
            if current_goal_distance > self.best_wall_distance_to_goal + 0.03:
                self.get_logger().warn("Anti-orbit escape -> GO_TO_GOAL")
                self.mode = "GO_TO_GOAL"
                return

        front = self.get_front_distance()
        right = self.get_right_distance()
        front_right = self.get_front_right_distance()
        back_right = self.get_back_right_distance()

        if math.isfinite(front_right) and math.isfinite(back_right):
            parallel_error = back_right - front_right
        else:
            parallel_error = 0.0

        if front < 0.62 or front_right < 0.46:
            self.publish_cmd(0.0, 0.38)
            return

        if (not math.isfinite(right)) or right > 1.10:
            self.publish_cmd(0.07, -0.10)
            return

        distance_error = self.desired_wall_distance - right
        w = self.kp_wall_dist * distance_error + self.kp_wall_parallel * parallel_error
        w = clamp(w, -0.38, 0.38)

        v = self.wall_follow_speed
        if front < 0.75:
            v = min(v, 0.05)

        self.publish_cmd(v, w)

    def rotate_to_final_yaw_step(self):
        goal_yaw = self.get_goal()[2]
        yaw_error = wrap(goal_yaw - self.yaw)

        if abs(yaw_error) <= self.yaw_tol:
            self.stop_robot()
            self.get_logger().info(
                f"Waypoint {self.current_waypoint_index + 1} reached with final yaw"
            )
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                self.mode = "DONE"
            else:
                self.mode = "GO_TO_GOAL"
            return

        self.publish_cmd(
            0.0,
            clamp(self.k_ang_yaw * yaw_error, -self.max_ang, self.max_ang),
        )

    def control_loop(self):
        if not self.got_odom:
            return

        self.log_path_sample()

        if self.mode == "DONE":
            self.stop_robot()
            self.save_path_log()
            self.get_logger().info("All waypoints completed")
            self.timer.cancel()
            return

        if self.mode == "FAILED":
            self.stop_robot()
            self.save_path_log()
            self.get_logger().warn("Controller stopped in FAILED state")
            self.timer.cancel()
            return

        if self.current_waypoint_index >= len(self.waypoints):
            self.mode = "DONE"
            return

        if self.mode == "GO_TO_GOAL":
            self.go_to_goal_step()
            return

        if self.mode == "ROTATE_TO_FINAL_YAW":
            self.rotate_to_final_yaw_step()
            return

        if self.mode == "ALIGN_RIGHT_WALL":
            self.align_obstacle_on_right_step()
            return

        if self.mode == "FOLLOW_RIGHT_BOUNDARY":
            self.follow_right_boundary_step()
            return


def main(args=None):
    rclpy.init(args=args)
    node = BugWaypointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
