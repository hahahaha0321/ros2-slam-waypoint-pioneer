import csv
from pathlib import Path
import matplotlib.pyplot as plt
import math

path_csv = Path.home() / "bug_path_log.csv"
waypoints_file = Path.home() / "ros_ws/waypoints.txt"

obstacles = {
    "cone1": (-4.5, 2.2),
    "cone2": (0.0, 3.2),
    "bucket1": (4.3, 1.8),
    "bucket2": (3.9, -1.8),
    "box1": (-3.8, -2.2),
    "box2": (0.6, -3.4),
}

walls = [
    ("w1", 7.0, 2.0, 2.03444, 4.47214),
    ("w2", 3.0, 5.0, 2.81984, 6.32456),
    ("w3", -3.0, 5.0, -2.81984, 6.32456),
    ("w4", -7.0, 2.0, -2.03444, 4.47214),
    ("w5", -7.0, -2.0, -1.10715, 4.47214),
    ("w6", -3.0, -5.0, -0.32175, 6.32456),
    ("w7", 3.0, -5.0, 0.32175, 6.32456),
    ("w8", 7.0, -2.0, 1.10715, 4.47214),
]

x_vals = []
y_vals = []

with open(path_csv) as f:
    reader = csv.DictReader(f)
    for row in reader:
        x_vals.append(float(row["x_m"]))
        y_vals.append(float(row["y_m"]))

if not x_vals:
    raise ValueError("No path data found in bug_path_log.csv")

waypoint_x = []
waypoint_y = []

with open(waypoints_file) as f:
    for line_num, line in enumerate(f, start=1):
        line = line.strip()

        if not line or line.startswith("#"):
            continue

        parts = line.replace(",", " ").split()

        if len(parts) < 2:
            raise ValueError(f"Invalid waypoint format on line {line_num}: {line}")

        x = float(parts[0])
        y = float(parts[1])

        waypoint_x.append(x)
        waypoint_y.append(y)

if not waypoint_x:
    raise ValueError("No waypoints found in waypoints.txt")

plt.figure(figsize=(8, 6))

for i, (_, cx, cy, yaw, length) in enumerate(walls):
    dx = (length / 2.0) * math.cos(yaw)
    dy = (length / 2.0) * math.sin(yaw)

    x1 = cx - dx
    y1 = cy - dy
    x2 = cx + dx
    y2 = cy + dy

    if i == 0:
        plt.plot([x1, x2], [y1, y2], "k-", linewidth=3, label="Oval Boundary")
    else:
        plt.plot([x1, x2], [y1, y2], "k-", linewidth=3)

plt.plot(x_vals, y_vals, linewidth=2, label="Robot Path")

for cx, cy, yaw, length in [(w[1], w[2], w[3], w[4]) for w in walls]:
    dx = (length / 2.0) * math.cos(yaw)
    dy = (length / 2.0) * math.sin(yaw)

    x1 = cx - dx
    y1 = cy - dy
    x2 = cx + dx
    y2 = cy + dy

    plt.scatter([x1, x2], [y1, y2], s=20, color="black")

step = max(1, len(x_vals) // 20)
for i in range(0, len(x_vals) - 1, step):
    dx = x_vals[i + 1] - x_vals[i]
    dy = y_vals[i + 1] - y_vals[i]
    plt.arrow(
        x_vals[i],
        y_vals[i],
        dx,
        dy,
        head_width=0.05,
        length_includes_head=True,
        alpha=0.6,
    )

plt.scatter(x_vals[0], y_vals[0], color="green", s=100, label="Start")
plt.scatter(x_vals[-1], y_vals[-1], color="red", s=100, label="End")

plt.scatter(waypoint_x, waypoint_y, color="purple", s=100, label="Waypoints")
plt.plot(waypoint_x, waypoint_y, "--", linewidth=1.5, alpha=0.7, label="Waypoint Route")

for i, (wx, wy) in enumerate(zip(waypoint_x, waypoint_y), start=1):
    plt.text(wx + 0.05, wy + 0.05, f"W{i}", fontsize=10)

obs_x = [p[0] for p in obstacles.values()]
obs_y = [p[1] for p in obstacles.values()]
plt.scatter(obs_x, obs_y, marker="s", s=140, color="orange", label="Obstacles")

for name, (ox, oy) in obstacles.items():
    plt.text(ox + 0.08, oy + 0.08, name, fontsize=9)

plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("Bug Algorithm Path with Waypoints and Obstacles")
plt.legend()
plt.axis("equal")
plt.grid(True)
plt.tight_layout()
plt.show()
