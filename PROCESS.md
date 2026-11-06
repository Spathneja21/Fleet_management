# Build Process Log — Fleet Management

Working notes on how this project is set up and built, in order. See [PLAN.md](PLAN.md) for the phases.
Background for everything copied here: `../AMR_Practice_project/PROCESS.md` (esp. §30–34, Gazebo).

---

## 0. Inherited lessons from AMR_Practice_project (Gazebo backend)

- All ROS 2 Humble work runs in Docker (`supermarketbot:humble`), host is too old.
- Gazebo Sim = Fortress (`ign gazebo`). Its `gpu_lidar` crashes the server here, so the robot is spawned
  **without sensors** (`scripts/urdf_gazebo_nosensors.py`) and `/scan` comes from `gazebo_lidar`
  (ray cast from the true pose against `models/gazebo/office/model.sdf` boxes).
- True pose comes from the URDF's `OdometryPublisher` plugin (`ground_truth/odom`, frame `world`).
- Don't set `LIBGL_ALWAYS_SOFTWARE` for Gazebo (breaks the GUI with GPU passthrough).
- `GZ_SIM_RESOURCE_PATH` needs **both** `.../models/gazebo` and `/ros2_ws/src`, otherwise meshes
  silently fail to load.
- Caster slip fix (`mu1`/`mu2` 0.01 on `chassis_link`) is already in the copied URDF — keep it.
- Isolate test runs with `IGN_PARTITION` + `ROS_DOMAIN_ID`; leftover servers share discovery on `--net=host`.
- Everything downstream needs `use_sim_time:=true`.

## 1. Files copied from AMR (2026-11-07)

From `AMR_Practice_project/src/amr_bot` into `src/fleet_manager`:

```
src/lidar_explorer.cpp  src/gazebo_lidar.cpp
urdf/supermarketbot.urdf
models/meshes/  models/gazebo/ (office.world.sdf + office/ model, ~21 MB)
scripts/urdf_gazebo_nosensors.py
config/async_slam_toolbox_cfg.yaml
launch/gazebo.launch.py
maps/office.pgm  maps/office.yaml
```

Edits made while copying:
- `amr_bot` -> `fleet_manager` in the URDF mesh URIs, `gazebo_lidar` default `model_sdf` path, launch file.
- `gazebo_camera` node removed from `gazebo.launch.py` (not copied: needs MuJoCo; not needed for mapping).
- New `CMakeLists.txt` / `package.xml`: only `lidar_explorer` + `gazebo_lidar`, no MuJoCo/GLFW deps.

Build check: `colcon build` in a throwaway `supermarketbot:humble` container -> 1 package finished
(12 s), installs `lidar_explorer`, `gazebo_lidar`, `urdf_gazebo_nosensors.py`. Note: `build/ install/ log/`
from that run are root-owned (container runs as root).

## 2. Container

The existing `amr` container mounts `AMR_Practice_project` at `/ros2_ws`. Hardcoded paths in the copied
files assume `/ros2_ws/src/fleet_manager/...`, so this project needs its own container:

```bash
xhost +local:docker
docker run -it --name fleet \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/projects/fleet_management:/ros2_ws \
  --device /dev/dri/card0 --device /dev/dri/renderD128 \
  --entrypoint bash \
  supermarketbot:humble
# inside
cd /ros2_ws && colcon build --symlink-install && source install/setup.bash
```

## 3. Single-robot Gazebo smoke test (headless)

Throwaway container, isolated (`IGN_PARTITION=fleettest`, `ROS_DOMAIN_ID=77`), `ign gazebo -s -r` + `gazebo.launch.py`:
all topics up (`/scan`, `/odom`, `/ground_truth/odom`, `/tf`, `/clock`, `/joint_states`), scan ranges sane
(~4.7–7.6 m), `/cmd_vel` 0.2 m/s moved the robot to true x = 1.15 m. SLAM + explorer not run yet in this project.
README rewritten from scratch; "Commands" section added.

## Open items

- Phase 2: namespaced single robot.
- `README.md` is still the AMR copy — rewrite in phase 8.
