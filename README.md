# rl_xarm

Reinforcement learning framework for the UFactory xArm6, built on ROS 2. The first
task is **tennis ball tracking**: the arm learns to keep a moving tennis ball
centered in its wrist camera frame. The ball's motion is randomized in simulation
so the policy generalizes to unpredictable targets.

The project was originally built on **Gazebo Classic** and has since been ported
to **Gazebo Ignition (Fortress / gz-sim 6)**. Both simulator paths are kept side
by side (see [Launch files](#launch-files)) — the Classic files are unmodified,
and the Ignition equivalents are separate files with a `_gz` suffix.

## Repository layout

```
rl_xarm/
├── xarm6_control_trainig/   # arm control, ball gantry, RL agent, launch files
│   ├── launch/              # Classic and Ignition launch files
│   ├── urdf/                # tennis-ball gantry + robot description (xacro)
│   ├── worlds/               # Ignition .sdf worlds (tennis_world, pedestal)
│   ├── config/                # ros_gz_bridge topic mapping (Ignition)
│   ├── scripts/               # ball motion, arm control, reset, RL agent
│   └── src/                   # controller_moveit.cpp (C++ joint controller)
└── image_processing/          # camera-based tennis ball tracker
    └── scripts/
```

(Package directory is spelled `xarm6_control_trainig` — a typo carried over from
the original package name; renaming it would break existing installs/imports,
so it's left as-is.)

## Dependencies

**System**
- Ubuntu 22.04
- ROS 2 Humble
- One simulator path (or both):
  - **Gazebo Classic 11** — `gazebo_ros`, `gazebo_ros_pkgs` (joint state publisher,
    joint pose trajectory, and ros_control plugins)
  - **Gazebo Ignition Fortress** (gz-sim 6) — `ros_gz_sim`, `ros_gz_bridge`,
    `ign_ros2_control`
- MoveIt 2 (`moveit_core`, `moveit_ros_planning`, `moveit_ros_planning_interface`,
  `moveit_msgs`, `interactive_markers`, `geometric_shapes`)
- [`xarm_ros2`](https://github.com/xArm-Developer/xarm_ros2) (Humble branch),
  cloned as a sibling package — provides `xarm_msgs`, `xarm_planner`,
  `xarm_description`, `xarm_moveit_config`, `xarm_controller`, `uf_ros_lib`
- `ros2_control` / `controller_manager`
- `xacro`, `robot_state_publisher`

**Python**
- `rclpy`
- `numpy` (2.x)
- `opencv-python` (`cv2`) — used directly for image decoding; `cv_bridge` is
  intentionally avoided in the sim-facing tracker since its compiled extension
  isn't compatible with NumPy 2.x
- `torch` — SAC (Soft Actor-Critic) agent
- `PyYAML`

Build with `colcon build` from your ROS 2 workspace root after cloning `rl_xarm`
and `xarm_ros2` into `src/`, then `source install/setup.bash`.

## Launch files

All launch files live in `xarm6_control_trainig/launch/`.

| File | Simulator | Spawns |
|---|---|---|
| `tennis_ball_gazebo_spawn.launch.py` | Classic | Ball entity only, via `gazebo_ros/spawn_entity.py` |
| `launch_tennis_ball_in_gazebo.launch.py` | Classic | Ball gantry (robot_state_publisher, spawn, joint controller, reset node, image tracker) |
| `launch_tennis_ball.launch.xml` | Classic | Includes `xarm_planner`'s `xarm6_planner_gazebo.launch.py` **and** `launch_tennis_ball_in_gazebo.launch.py` — arm + ball together |
| `launch_xarm6_tennis_ball_in_gazebo.launch.py` | Classic | Same as above, via Python includes with a 5s delay so the arm comes up first |
| `launch_tennis_ball_in_gz.launch.py` | Ignition | Ignition world, ball gantry `robot_state_publisher`, spawn via `ros_gz_sim create`, `ros_gz_bridge` (clock, joint states, `/set_joint_trajectory`) |
| `launch_xarm6_tennis_ball_in_gz.launch.py` | Ignition | Full stack: includes the file above, then spawns the xArm6 (`ign_ros2_control`) + pedestal, loads controllers, optionally starts MoveIt (`use_moveit:=true` by default), camera bridge, and the arm-command helper nodes |

Useful launch arguments on the Ignition top-level launch:
- `use_moveit` (default `true`) — start `move_group` + `xarm_planner_node`
- `use_moveit_rviz` (default `false`) — show RViz
- `enable_arm_command_helpers` (default `true`) — run `xarm_joint_controller.py`
  / `reset_sim_tennis_ball.py` (these require the MoveIt services above)
- `show_frame` (default `false`) — render the (non-physical) black gantry frame

## Scripts (`xarm6_control_trainig/scripts/`)

- **`tennis_ball_position_publisher_joint_cylinder_velocity.py`** — the ball
  mover used by the current cylindrical gantry (`theta_joint`/`r_joint`/`z_joint`).
  Each joint independently seeks a random target within its limits at a random
  speed, giving even coverage of the reachable volume via `/set_joint_trajectory`.
- **`tennis_ball_position_publisher_joint_position.py`** /
  **`tennis_ball_position_publisher_joint_velocity.py`** — earlier ball-mover
  versions for an XYZ-gantry model (random walk / sinusoidal), kept for
  reference.
- **`xarm_joint_controller.py`** — subscribes to `xarm_joint_controller`
  (joint-angle array) and drives the arm via the `xarm_planner` services
  `/xarm_joint_plan` + `/xarm_exec_plan`.
- **`reset_sim_tennis_ball.py`** — on `/reset_sim`, publishes a fixed "home"
  joint pose to `xarm_joint_controller` and zeros the ball gantry via
  `/set_joint_trajectory`.
- **`soft_actor_critic_agent.py`** — the RL agent (PyTorch SAC). Currently a
  skeleton: publisher and state-processing scaffolding are in place, but the
  network/replay-buffer/training loop are not yet implemented.
- **`src/controller_moveit.cpp`** (built as the `controller_moveit` executable)
  — C++ reimplementation of the joint controller for lower-latency plan/execute
  calls to `/xarm_joint_plan` and `/xarm_exec_plan`.

## Image processing (`image_processing/scripts/`)

- **`tennis_ball_tracker_sim_publisher.py`** — the tracker used in launch files.
  Subscribes to `/color/image_raw` and `/aligned_depth_to_color/image_raw`,
  HSV-thresholds + contour-detects the ball, and publishes its 2D pixel offset
  from image center plus depth on `/ball_position`. Decodes ROS `Image`
  messages manually (no `cv_bridge`, see NumPy 2.x note above).
- **`tennis_ball_tracker_sim.py`** — same tracking logic, visualization-only
  (uses `cv_bridge`; not used by any launch file).
- **`tennis_ball_colour_calib_sim.py`** — interactive HSV threshold tuner
  (OpenCV trackbars) for calibrating the color mask above.
- **`tennis_ball_depth_calib_sim.py`** — visualizes the depth image with a
  colormap for calibrating the depth reading.

## Typical usage

Ignition, arm + ball, full MoveIt stack:
```bash
ros2 launch xarm6_control_trainig launch_xarm6_tennis_ball_in_gz.launch.py
```

Classic, arm + ball:
```bash
ros2 launch xarm6_control_trainig launch_tennis_ball.launch.xml
```

Ball only (Ignition), for iterating on the tracker/motion generator without the
arm:
```bash
ros2 launch xarm6_control_trainig launch_tennis_ball_in_gz.launch.py
```
