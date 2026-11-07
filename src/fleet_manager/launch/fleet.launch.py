"""Three robots in the Gazebo office, each in its own namespace (robot1..3, TF prefix robotN/).
Always: spawn (sensor-stripped URDF), robot_state_publisher, gazebo_lidar per robot; one ros_gz_bridge for
/clock, the shared /tf and every robot's topics; RViz.

Two modes:
- explore (default): per robot slam_toolbox + lidar_explorer. A static TF world -> robotN/map (= spawn pose)
  places each SLAM map, map_merger fuses them into /map (frame world), every explorer plans on that map.
- navigate (map:=<saved map yaml>): one map_server publishes the saved map as /map (frame world). Per robot
  AMCL (starts at the spawn pose) + nav2 (planner, controller, behaviors, bt_navigator). goal_dispatcher
  sends each RViz "2D Goal Pose" to the robot with the shortest planned path.

Start `ign gazebo -r .../office.world.sdf` yourself first (interactive GUI window). Then:
    ros2 launch fleet_manager fleet.launch.py                       # explore
    ros2 launch fleet_manager fleet.launch.py map:=<path>.yaml      # navigate
    explore:=false: no explorers, rviz:=false: no RViz
"""
import os

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# (name, x, y, yaw) in the Gazebo world frame, spread over the centre hall, left wing and right wing.
ROBOTS = [
    ('robot1', 0.0, 0.0, 0.0),
    ('robot2', -9.0, 2.0, 0.0),
    ('robot3', 19.0, -1.0, 3.14159),
]
NAV2_NODES = {'amcl': 'nav2_amcl', 'planner_server': 'nav2_planner', 'controller_server': 'nav2_controller',
              'behavior_server': 'nav2_behaviors', 'bt_navigator': 'nav2_bt_navigator'}  # node -> package


def nav2_params_file(share, name, x, y, yaw):
    """config/nav2_params.yaml for one robot: <robot> filled in, nested under its namespace, AMCL at spawn."""
    with open(os.path.join(share, 'config', 'nav2_params.yaml')) as f:
        params = yaml.safe_load(f.read().replace('<robot>', name))
    params['amcl']['ros__parameters']['initial_pose'] = {'x': x, 'y': y, 'z': 0.0, 'yaw': yaw}
    path = f'/tmp/nav2_params_{name}.yaml'
    with open(path, 'w') as f:
        yaml.safe_dump({name: params}, f)
    return path


def setup(context):
    share = get_package_share_directory('fleet_manager')
    urdf_script = os.path.join(get_package_prefix('fleet_manager'), 'lib', 'fleet_manager', 'urdf_gazebo_nosensors.py')
    with open(os.path.join(share, 'urdf', 'supermarketbot.urdf')) as f:
        robot_description = f.read()
    with open(os.path.join(share, 'config', 'async_slam_toolbox_cfg.yaml')) as f:
        slam_params = yaml.safe_load(f)['slam_toolbox']['ros__parameters']
    map_yaml = LaunchConfiguration('map').perform(context)
    navigate = bool(map_yaml)

    actions = []
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

        if navigate:
            # nav2 nodes started directly: nav2_bringup's navigation_launch.py remaps /tf -> tf, which would
            # move each robot's TF into its namespace, but this fleet shares one /tf.
            params = nav2_params_file(share, name, x, y, yaw)
            for node, pkg in NAV2_NODES.items():
                actions.append(Node(package=pkg, executable=node, namespace=name, output='screen', parameters=[params]))
            actions.append(Node(
                package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager',
                namespace=name, output='screen',
                parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': list(NAV2_NODES)}]))
        else:
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
                condition=IfCondition(LaunchConfiguration('explore')), remappings=[('map', '/map')],
                parameters=[{'map_frame': 'world', 'base_frame': f'{name}/base_link', 'use_sim_time': True}]))

    if navigate:
        actions.append(Node(package='nav2_map_server', executable='map_server', output='screen',
                            parameters=[{'yaml_filename': map_yaml, 'frame_id': 'world', 'use_sim_time': True}]))
        actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                            name='lifecycle_manager_map', output='screen',
                            parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server']}]))
        actions.append(Node(package='fleet_manager', executable='goal_dispatcher', output='screen',
                            parameters=[{'robots': names, 'use_sim_time': True}]))
    else:
        actions.append(Node(package='fleet_manager', executable='map_merger', output='screen',
                            parameters=[{'robots': names, 'use_sim_time': True}]))
    # map, robot models, paths and goals preconfigured (config/fleet.rviz)
    actions.append(Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
                        arguments=['-d', os.path.join(share, 'config', 'fleet.rviz')],
                        parameters=[{'use_sim_time': True}]))
    actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=bridge_args, output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='', description='saved map yaml -> navigation mode'),
        DeclareLaunchArgument('explore', default_value='true', description='explore mode: run the explorers'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=setup),
    ])
