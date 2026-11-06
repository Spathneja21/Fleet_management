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

## 4. Phases 2–4: namespaced robots, fleet launch, robots see each other (2026-11-07)

**Naming:** topics under `/robotN/...`, TF frames prefixed `robotN/`. Shared: `/clock`, `/tf`, `/tf_static`.

- `scripts/urdf_gazebo_nosensors.py [ns]`: with a namespace, rewrites plugin `<topic>`/`<odom_topic>` to
  `/robotN/...` (cmd_vel, odom, joint_states, ground_truth/odom), ground-truth `tf_topic`, and
  `frame_id`/`child_frame_id`/`robot_base_frame` to `robotN/...`. `/tf` stays shared, `odom_frame world`
  untouched. Without an argument the output is identical to the AMR script (diffed).
- `gazebo_lidar`: relative topics (`scan`, `ground_truth/odom`), frame `robotN/lidar_link` from the node
  namespace, new param `other_robots` -> subscribes to their `/robotX/ground_truth/odom` and ray casts a
  disc (`robot_radius` 0.13 m, chassis 0.16x0.18 + wheels) per robot.
- `lidar_explorer`: relative topics (`scan`, `odom`, `map`, `cmd_vel`, `explorer_*`), new params
  `map_frame` / `base_frame`. Algorithm unchanged.
- `launch/fleet.launch.py` (replaces `gazebo.launch.py`): per robot spawn (`-x -y -Y`), RSP
  (`frame_prefix robotN/`), gazebo_lidar, slam_toolbox, lidar_explorer (`explore:=false` to skip);
  one `parameter_bridge` for everything.
- **slam_toolbox in a namespace:** its map topic is the `map_name` param, default absolute `/map` —
  three instances would collide. Set `map_name: /robotN/map` + frames + `scan_topic` per robot. The yaml
  is loaded in the launch file and passed as a dict, because its `slam_toolbox:` key doesn't match the
  namespaced node name `/robotN/slam_toolbox`.
- **Spawn poses** (`ROBOTS` in the launch file), picked from `maps/office.pgm` rendered with a grid
  (AMR map frame = Gazebo world, the AMR robot spawned at the origin): robot1 (0,0,0) centre hall,
  robot2 (-9,2,0) left wing, robot3 (19,-1,π) right wing.

**Checks (headless, isolated `IGN_PARTITION`/`ROS_DOMAIN_ID`):**
- `explore:=false`: all `/robotN/*` topics up, true poses = spawn poses, three separate maps
  (368x272, 273x328, 356x256), TF `robot2/map -> robot2/lidar_link` resolves, scan frame `robot3/lidar_link`.
- Disc test: two extra lidar nodes at robot2's pose, noise 0, with/without `other_robots:=[robot1]`.
  Exactly 2 beams differ (167, 168): 9.10 m vs inf. Expected: |(0,0)-(-9,2)| - 0.13 = 9.09 m at bearing
  -12.5° (beam 167 = -13°). ✓
- Full run, 265 s sim: all three drive simultaneously; maps 175 / 185 / 133 m^2; 5 stuck events, all
  recovered. **But** they converge: robot1 and robot2 both went east, robot3 came back to the centre,
  all within x 10–24 by the end — each robot only knows its own map. That is phases 5–6.

## Open items

- Phase 5: `map_merger` (world frame, merged `/map`).
- Phase 6: explorers on the merged map + goal reservation.
- `README.md` is still the AMR copy — rewrite in phase 8.
