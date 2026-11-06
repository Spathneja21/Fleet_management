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

### Run: three robots
Run each command in its own terminal, inside the container. Don't set `LIBGL_ALWAYS_SOFTWARE`.
```bash
# 1. world (-r starts it running)
export GZ_SIM_RESOURCE_PATH=/ros2_ws/src/fleet_manager/models/gazebo:/ros2_ws/src
ign gazebo -r /ros2_ws/src/fleet_manager/models/gazebo/office.world.sdf
# 2. three robots: spawn, bridge, TF, lidar, SLAM and explorer for each
ros2 launch fleet_manager fleet.launch.py          # explore:=false -> robots + SLAM only
# 3. visualisation (Fixed Frame: robot1/map, add Map displays for /robotN/map)
rviz2 --ros-args -p use_sim_time:=true
```

Each robot lives in its own namespace (`robot1`..`robot3`) with TF prefix `robotN/`. Spawn poses are
set in `ROBOTS` at the top of `launch/fleet.launch.py`.

Drive one robot by hand (with `explore:=false`):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot1/cmd_vel
```

Current state: each robot explores with **its own** map, so they still drift into the same areas.
The shared map and frontier coordination come next.

## Architecture

_To be added, diagram by diagram._
