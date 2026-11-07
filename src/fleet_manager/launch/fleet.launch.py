"""Three robots in the Gazebo office, each in its own namespace (robot1..3, TF prefix robotN/):
spawn (sensor-stripped URDF), robot_state_publisher, gazebo_lidar, slam_toolbox, lidar_explorer.
One ros_gz_bridge carries /clock, the shared /tf and every robot's topics. A static TF world -> robotN/map
(= spawn pose) places each SLAM map, map_merger fuses them into /map (frame world) and every explorer
plans on that shared map.

Start `ign gazebo -r .../office.world.sdf` yourself first (interactive GUI window). Then:
    ros2 launch fleet_manager fleet.launch.py              # explore:=false to only bring up robots + SLAM
"""
import os

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# (name, x, y, yaw) in the Gazebo world frame, spread over the centre hall, left wing and right wing.
ROBOTS = [
    ('robot1', 0.0, 0.0, 0.0),
    ('robot2', -9.0, 2.0, 0.0),
    ('robot3', 19.0, -1.0, 3.14159),
]


def generate_launch_description():
    share = get_package_share_directory('fleet_manager')
    urdf_script = os.path.join(get_package_prefix('fleet_manager'), 'lib', 'fleet_manager', 'urdf_gazebo_nosensors.py')
    with open(os.path.join(share, 'urdf', 'supermarketbot.urdf')) as f:
        robot_description = f.read()
    with open(os.path.join(share, 'config', 'async_slam_toolbox_cfg.yaml')) as f:
        slam_params = yaml.safe_load(f)['slam_toolbox']['ros__parameters']

    explore = LaunchConfiguration('explore')
    actions = [DeclareLaunchArgument('explore', default_value='true')]
    bridge_args = ['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                   '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V']
    names = [r[0] for r in ROBOTS]

    for name, x, y, yaw in ROBOTS:
        bridge_args += [
            f'/{name}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            f'/{name}/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            f'/{name}/ground_truth/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            f'/{name}/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ]
        # ros_gz_sim create is a one-shot CLI tool, not a long-running node
        actions.append(ExecuteProcess(
            cmd=['bash', '-c',
                 f'python3 {urdf_script} {name} > /tmp/{name}.urdf && ros2 run ros_gz_sim create -world office '
                 f'-file /tmp/{name}.urdf -name {name} -x {x} -y {y} -z 0.05 -Y {yaw}'],
            output='screen'))
        actions.append(Node(
            package='robot_state_publisher', executable='robot_state_publisher', namespace=name,
            parameters=[{'robot_description': robot_description, 'frame_prefix': f'{name}/', 'use_sim_time': True}]))
        actions.append(Node(
            package='fleet_manager', executable='gazebo_lidar', namespace=name, output='screen',
            parameters=[{'other_robots': [n for n in names if n != name], 'use_sim_time': True}]))
        actions.append(Node(
            package='slam_toolbox', executable='async_slam_toolbox_node', namespace=name, output='screen',
            parameters=[{**slam_params,
                         'odom_frame': f'{name}/odom', 'map_frame': f'{name}/map', 'base_frame': f'{name}/base_link',
                         'scan_topic': f'/{name}/scan', 'map_name': f'/{name}/map', 'use_sim_time': True}]))
        # SLAM starts its map at the robot's start pose, so world -> robotN/map is the spawn pose
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher', name=f'{name}_map_tf',
            arguments=['--x', str(x), '--y', str(y), '--yaw', str(yaw),
                       '--frame-id', 'world', '--child-frame-id', f'{name}/map']))
        actions.append(Node(
            package='fleet_manager', executable='lidar_explorer', namespace=name, output='screen',
            condition=IfCondition(explore), remappings=[('map', '/map')],
            parameters=[{'map_frame': 'world', 'base_frame': f'{name}/base_link', 'use_sim_time': True}]))

    actions.append(Node(package='fleet_manager', executable='map_merger', output='screen',
                        parameters=[{'robots': names, 'use_sim_time': True}]))
    actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=bridge_args, output='screen'))
    return LaunchDescription(actions)
