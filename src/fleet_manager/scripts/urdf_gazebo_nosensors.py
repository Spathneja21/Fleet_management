#!/usr/bin/env python3
"""Robot URDF for spawning into Gazebo Sim, optionally namespaced for a multi-robot fleet.

1. Strips the lidar/camera <gazebo reference="..."> sensor blocks: Gazebo's gpu_lidar/camera crash the
   gz-sim server on this machine (AMR_Practice_project PROCESS.md 30-34); /scan comes from gazebo_lidar.
2. With a namespace (e.g. robot1), rewrites the plugin topics to /robot1/... and the TF frames to
   robot1/odom -> robot1/base_link, so several copies can share one world and one /tf.

Regenerated from the real URDF every time, so it can't drift from the robot's actual geometry.
Usage: urdf_gazebo_nosensors.py [namespace] > /tmp/robot.urdf
"""
import re
import sys
from pathlib import Path

URDF = Path(__file__).resolve().parent.parent / "urdf" / "supermarketbot.urdf"
if not URDF.exists():  # installed copy: lib/fleet_manager/ -> share/fleet_manager/urdf
    URDF = Path(__file__).resolve().parents[2] / "share" / "fleet_manager" / "urdf" / "supermarketbot.urdf"

s = URDF.read_text()
for link in ("lidar_link", "camera_link"):
    s = re.sub(rf'<gazebo reference="{link}">.*?</gazebo>\n', "", s, flags=re.S)

if len(sys.argv) > 1:
    ns = sys.argv[1].strip("/")
    for tag in ("topic", "odom_topic"):  # cmd_vel, odom, joint_states, ground_truth/odom (/tf stays shared)
        s = re.sub(rf"<{tag}>/?([^<]+)</{tag}>", rf"<{tag}>/{ns}/\1</{tag}>", s)
    s = s.replace("<tf_topic>/ground_truth/tf</tf_topic>", f"<tf_topic>/{ns}/ground_truth/tf</tf_topic>")
    for tag in ("frame_id", "child_frame_id", "robot_base_frame"):
        s = re.sub(rf"<{tag}>([^<]+)</{tag}>", rf"<{tag}>{ns}/\1</{tag}>", s)

sys.stdout.write(s)
