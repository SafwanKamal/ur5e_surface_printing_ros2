# UR5e Surface Printing — ROS 2 Jazzy

Research workspace for a UR5e with a syringe tool: MoveIt planning, collision
models, offline toolpath import, bounded robot/pump execution, mock rehearsal and
RealSense cloud processing. The repository root is the workspace (`~/ur5e_ws`);
ROS packages live in `src/`.

## Start here

| Task | Guide |
|---|---|
| Build, launch, inspect, change paths, operate syringe, stop | [Complete command guide](docs/COMMANDS.md) |
| Every declared ROS executable parameter and default | [Parameter reference](docs/PARAMETERS.md) |
| First physical bring-up and 5 mm motion-only rehearsal | [Hardware startup](docs/HARDWARE_STARTUP.md) |
| Plan/check/rehearse/print one surface line | [Demo runbook](docs/DEMO_RUNBOOK.md) |
| Manually positioned constant-orientation helix | [Tube demo](TUBE_DEMO.md) |
| Mega upload, ramps, enable pin and watchdog commands | [Firmware guide](docs/SYRINGE_FIRMWARE.md) |
| Pump CAD alignment and TCP calibration | [Pump model](docs/PUMP_MODEL.md) |
| Python planner/executor TCP speed verification | [TCP speed checks](docs/TCP_SPEED_CHECK.md) |
| Current Wrist 3 limits | [Wrist limits](docs/WRIST_SIMULATION_RANGE.md) |

## Current state

`main` integrates the former `demo/coarse-collision` development line (27 commits
beyond the previous main), including all `demo/syringe-print` work. Use `main` for
new work. Branch names in old conversations refer to this integrated history.

| Component | Implemented behavior |
|---|---|
| `surface_printing` | Capture line/circle/square or a downward-facing tube helix; convert registered CSVs; plan one line; save/check/replay a v2 plan; preflight hardware |
| `syringe_interfaces` | Pulse/volume actions and extrusion/flow services |
| `syringe_controller` | Exclusive serial access, calibrated pulse estimates, leased flow, heartbeat, latched faults and STOP |
| `ur5e_toolheads` | Pump STL, coarse collision envelopes, configurable TCP, CAD-extension and axial-nozzle configuration tools |
| `ur5e_probe_moveit_config` | Mock stack and real driver/MoveIt/RViz launch; robot-specific kinematics file required for real launch |
| `ur5e_moveit_cpp` | Legacy motion experiments, collision-scene tools, IK checks, single/multiple raster-line planning and guarded segment execution |
| `ur5e_perception` | Point-cloud cropping/filtering, table removal, principal object cluster and centroid/bounds |
| `firmware/syringe_demo` | Mega PUL=28, DIR=27, ENA=25; ramps, idle disable, bounded commands and configurable serial heartbeat |

The companion [geometry pipeline](https://github.com/SafwanKamal/Surface_printing_path_generation_pipeline)
generates millimeter CSVs in the object frame. Convert once to world meters for
the Python planner; the C++ raster tools instead consume the original millimeter
CSV and apply an explicit object pose.

## Install and build

On Ubuntu 24.04 with ROS 2 Jazzy installed:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep \
  ros-jazzy-moveit ros-jazzy-ur ros-jazzy-ur-calibration \
  ros-jazzy-pcl-ros ros-jazzy-pcl-conversions
# Optional camera acquisition:
sudo apt install ros-jazzy-realsense2-camera

git clone https://github.com/SafwanKamal/ur5e_surface_printing_ros2.git ~/ur5e_ws
cd ~/ur5e_ws
source /opt/ros/jazzy/setup.bash
# Initialize rosdep once if it has not been configured on this computer.
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

For an existing clone, preserve local calibration edits, then `git fetch origin`,
`git switch main`, and `git pull --ff-only origin main` from the repository root.
A divergent local branch needs reconciliation; do not discard lab changes.

Mock preview, in separate terminals with the same sourced environment:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur5e_ws/install/setup.bash
export ROS_DOMAIN_ID=78
ros2 launch ur5e_probe_moveit_config demo.launch.py
```

Follow [COMMANDS.md](docs/COMMANDS.md) for actual capture/planning and execution.
Physical examples use domain 77; mock examples use 78. Stop physical drivers and
disconnect pump power for mock work. Domain separation alone is not isolation.

## Important current configuration

The committed `printing_tool.yaml` was preserved during cleanup. It currently
contains `enabled: true`, `calibrated: true`, `coarse_collision: true` and a
**200 mm CAD-axis needle extension**. The 160 mm axial nozzle in the tube guide
is an alternate configuration you must explicitly measure and apply; it is not
the committed default. A YAML calibration flag is not evidence that a different
physical assembly is calibrated. TCP setters reset that flag after editing.

Wrist 3 currently permits ±180° in both mock and physical Python checks. Review
actual pump/needle clearance and allowed-collision pairs. The firmware enable
polarity is provisionally LOW and must match your driver. Normal idle disables
after 300 ms, removing holding torque.

## Validation and limitations

Offline verification uses 59 Python tests, a C++ firmware host shim, Python
syntax checks and XML parsing. Run the commands in [COMMANDS.md](docs/COMMANDS.md#validation).
A host shim cannot validate AVR timing, driver polarity or physical motor motion.
This cleanup environment does not contain ROS Jazzy/MoveIt or the robot; a
`colcon` build, launch checks, AVR upload and end-to-end stop tests remain lab checks.

The system has no automatic mesh-to-physical-object registration, hand-eye
calibration, encoder/force/flow feedback, cumulative stroke sensing, clog sensing,
or closed-loop bead control. Perception centroid/bounds are not full CAD pose
registration. Constant commanded flow is not a guarantee of bead width or lumen
geometry. Collision/TCP speed checks are sampled rather than continuous proofs.
Several legacy C++ demos execute immediately after planning; see their labels in
the command guide. The bounded Python workflow is the documented first-hardware
route. The C++ multi-line executor has a physical mode but requires independent
registration and speed validation before lab use.
