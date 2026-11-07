#!/usr/bin/env python3
"""Plot each robot's true path (ground_truth/odom) vs its SLAM-estimated path (explorer_path, already in
the world frame) over the final merged map, and report drift = distance between them over time.

Record a run first:
    ros2 bag record -o runs/run1 /map \
      /robot1/ground_truth/odom /robot2/ground_truth/odom /robot3/ground_truth/odom \
      /robot1/explorer_path /robot2/explorer_path /robot3/explorer_path

Then:
    python3 plot_trajectories.py runs/run1 --out runs/run1_trajectories.png

Needs: rosbag2_py + the message types (sourced ROS 2 env), numpy, matplotlib.
"""
import argparse

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.serialization import deserialize_message

ROBOTS = ("robot1", "robot2", "robot3")
COLORS = {"robot1": "tab:red", "robot2": "tab:green", "robot3": "tab:blue"}


def read_bag(path):
    """-> dict topic -> [(t_sec, msg), ...], decoded by each topic's registered type."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
    types = {"nav_msgs/msg/Odometry": Odometry, "nav_msgs/msg/Path": Path, "nav_msgs/msg/OccupancyGrid": OccupancyGrid}
    topic_type = {t.name: types[t.type] for t in reader.get_all_topics_and_types() if t.type in types}
    out = {t: [] for t in topic_type}
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic in topic_type:
            out[topic].append((t * 1e-9, deserialize_message(data, topic_type[topic])))
    return out


def drift(true_t, true_xy, est_t, est_xy):
    """Estimated path resampled onto the true path's timestamps (nearest), per-sample distance."""
    idx = np.searchsorted(est_t, true_t)
    idx = np.clip(idx, 1, len(est_t) - 1)
    left, right = est_t[idx - 1], est_t[idx]
    idx = np.where(true_t - left < right - true_t, idx - 1, idx)
    return np.hypot(*(true_xy - est_xy[idx]).T)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="rosbag2 directory recorded as shown above")
    ap.add_argument("--out", default=None, help="output PNG (default: <bag>_trajectories.png)")
    args = ap.parse_args()
    out = args.out or args.bag.rstrip("/") + "_trajectories.png"

    data = read_bag(args.bag)
    m = data.get("/map", [(None, None)])[-1][1]
    fig, ax = plt.subplots(figsize=(14, 8))
    if m is not None:
        grid = np.array(m.data, np.int16).reshape(m.info.height, m.info.width)
        img = np.where(grid < 0, 0.8, np.where(grid >= 50, 0.0, 1.0))
        extent = [m.info.origin.position.x, m.info.origin.position.x + m.info.width * m.info.resolution,
                  m.info.origin.position.y, m.info.origin.position.y + m.info.height * m.info.resolution]
        ax.imshow(img, cmap="gray", origin="lower", extent=extent, vmin=0, vmax=1)
    else:
        print("no /map in the bag, plotting paths only")

    print(f"{'robot':8} {'true (m)':>9} {'slam (m)':>9} {'mean drift':>11} {'max drift':>10}")
    for r in ROBOTS:
        c = COLORS[r]
        gt = data.get(f"/{r}/ground_truth/odom", [])
        est_msgs = data.get(f"/{r}/explorer_path", [])
        if not gt or not est_msgs:
            print(f"{r:8} no data (not in this bag)")
            continue
        true_t = np.array([t for t, _ in gt])
        true_xy = np.array([[m.pose.pose.position.x, m.pose.pose.position.y] for _, m in gt])
        # explorer_path is republished whole every 0.5 s and only grows: the last message has every point,
        # each stamped at the time it was recorded (see lidar_explorer.cpp on_odom)
        last_path = est_msgs[-1][1]
        est_t = np.array([p.header.stamp.sec + p.header.stamp.nanosec * 1e-9 for p in last_path.poses])
        est_xy = np.array([[p.pose.position.x, p.pose.position.y] for p in last_path.poses])

        ax.plot(true_xy[:, 0], true_xy[:, 1], color=c, lw=1.2, ls="--", alpha=0.6, label=f"{r} true")
        ax.plot(est_xy[:, 0], est_xy[:, 1], color=c, lw=2, label=f"{r} SLAM")
        ax.plot(*true_xy[-1], "o", color=c, ms=8)

        mask = (true_t >= est_t[0]) & (true_t <= est_t[-1])
        d = drift(true_t[mask], true_xy[mask], est_t, est_xy)
        true_len = np.sum(np.hypot(*np.diff(true_xy, axis=0).T))
        est_len = np.sum(np.hypot(*np.diff(est_xy, axis=0).T))
        print(f"{r:8} {true_len:9.1f} {est_len:9.1f} {d.mean():9.2f} m {d.max():8.2f} m")

    ax.legend(fontsize=8, loc="upper right")
    ax.set_title("True path (dashed) vs SLAM-estimated path (solid) over the merged map")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    fig.savefig(out, dpi=100, bbox_inches="tight")
    print(f"saved {out}")


if __name__ == "__main__":
    main()
