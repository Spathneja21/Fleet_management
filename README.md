# Fleet Management

Three robots explore an office in Gazebo at the same time, build one shared map, and track their trajectories.

ROS 2 Humble, C++, Gazebo Sim (Fortress). The exploration algorithm comes from
[`AMR_Practice_project`](../AMR_Practice_project).

## Commands

Everything runs inside Docker, in the `supermarketbot:humble` image (ROS 2 Humble, slam_toolbox, Gazebo Fortress).

### One-time container setup
```bash
xhost +local:docker          # repeat after every host login
docker run -it --name fleet \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/projects/fleet_management:/ros2_ws \
  --device /dev/dri/card0 --device /dev/dri/renderD128 \
  --entrypoint bash \
  supermarketbot:humble
```

### Every session
```bash
docker start -ai fleet       # or a second shell: docker exec -it fleet bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install && source install/setup.bash
```

### Run: single robot (current state)
Run each command in its own terminal, inside the container. Don't set `LIBGL_ALWAYS_SOFTWARE`.
```bash
# 1. world (-r starts it running)
export GZ_SIM_RESOURCE_PATH=/ros2_ws/src/fleet_manager/models/gazebo:/ros2_ws/src
ign gazebo -r /ros2_ws/src/fleet_manager/models/gazebo/office.world.sdf
# 2. robot, ROS bridge, TF, lidar
ros2 launch fleet_manager gazebo.launch.py
# 3. SLAM
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  --params-file /ros2_ws/src/fleet_manager/config/async_slam_toolbox_cfg.yaml
# 4. autonomous exploration
ros2 run fleet_manager lidar_explorer --ros-args -p use_sim_time:=true
# 5. visualisation
rviz2 --ros-args -p use_sim_time:=true
# when the explorer logs "exploration complete"
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/src/fleet_manager/maps/office_explored
```

Quick check without the explorer: `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"`.

### Run: three robots
_To be added (`fleet.launch.py`)._

## Architecture

_To be added, diagram by diagram._
