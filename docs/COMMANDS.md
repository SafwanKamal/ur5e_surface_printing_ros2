# System command guide

Commands below assume the repository root is `~/ur5e_ws`. Run each long-lived
launch/node in its own terminal. Unless marked otherwise, commands require a
running robot-description/MoveIt stack. Do not run concurrent command clients.
All IPs, object poses, table dimensions, nozzle measurements and material flows
are examples to replace with measured values.

## Update, build and select the environment

```bash
cd ~/ur5e_ws
git status --short
git fetch origin
git switch main
git pull --ff-only origin main
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Build a subset with its workspace dependencies, avoiding a missing generated
`syringe_interfaces` package on a fresh workspace:

```bash
colcon build --symlink-install --packages-up-to surface_printing ur5e_moveit_cpp
```

For memory-constrained builds:

```bash
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build \
  --symlink-install --executor sequential
```

In **every** terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur5e_ws/install/setup.bash
export ROS_DOMAIN_ID=78   # mock; use 77 for the physical lab stack
```

## Desktop console

The same bounded workflows are now available in a desktop GUI. Source the built
workspace first, then:

```bash
sudo apt install python3-tk
colcon build --symlink-install --packages-up-to surface_printing_gui
source install/setup.bash
ros2 run surface_printing_gui printing_gui \
  --workspace "$HOME/ur5e_ws" \
  --pipeline "$HOME/Surface_printing_path_generation_pipeline"
```

It starts in mock mode and requires a successful plan/check of the unchanged job
before execution. See [GUI.md](GUI.md) for included operations, process ownership,
stop behavior and advanced workflows that still use the CLI. RViz remains the 3D
preview. No hardware starts automatically when opening the console.

## Launch and inspect

Mock robot, MoveIt, controllers and RViz (pump power disconnected, real drivers stopped):

```bash
ros2 launch ur5e_probe_moveit_config demo.launch.py
```

Physical robot: extract calibration once using the actual robot IP, then use it
on every real launch. See [hardware startup](HARDWARE_STARTUP.md) before motion.

```bash
ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:=192.168.1.102 target_filename:="$HOME/ur5e_calibration.yaml"
ros2 launch ur5e_probe_moveit_config real_robot.launch.py \
  robot_ip:=192.168.1.102 reverse_ip:=192.168.1.100 \
  kinematics_params_file:="$HOME/ur5e_calibration.yaml" launch_rviz:=true
```

Inspection (no motion):

```bash
ros2 node list
ros2 topic list
ros2 action list -t
ros2 service list -t
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 topic echo /speed_scaling_state_broadcaster/speed_scaling --once
ros2 run tf2_ros tf2_echo world probe_tcp
ros2 run tf2_ros tf2_echo tool0 probe_tcp
ros2 launch ur5e_probe_moveit_config real_robot.launch.py --show-args
ros2 pkg executables surface_printing
ros2 pkg executables ur5e_moveit_cpp
ros2 param list /move_group
```

`tf2_echo` stays running until Ctrl-C. On physical hardware, controller
`scaled_joint_trajectory_controller` must be active; mock uses
`ur_manipulator_controller`. Planning group is `ur_manipulator`, frame `world`,
printing link `probe_tcp`. Early demos use `tool0`, which is a different TCP.

Read-only physical preflight:

```bash
ros2 run surface_printing hardware_preflight
```

## Tool configuration

Active source: `src/ur5e_toolheads/config/printing_tool.yaml`. Preserve a local
copy outside the repository before editing. Historical snapshots are under
`docs/archive/tool-config/` and are not installed.

CAD-axis extension from syringe outlet (preview, then apply only if appropriate):

```bash
ros2 run ur5e_toolheads set_needle_tcp \
  --config "$HOME/ur5e_ws/src/ur5e_toolheads/config/printing_tool.yaml" \
  --needle-length-mm 200
# Repeat with --write to apply the measured configuration.
```

Alternative straight axial nozzle, total tool0/flange-face-to-tip length (not
an additional CAD outlet extension), with a measured OUTER radius:

```bash
ros2 run ur5e_toolheads set_axial_nozzle_tcp \
  --config "$HOME/ur5e_ws/src/ur5e_toolheads/config/printing_tool.yaml" \
  --length-mm 160 --radius-mm 1
# 1 mm is only a syntax example. Repeat with measured radius and --write.
```

Both setters are preview-only until `--write`; they reset `calibrated` to false.
Rebuild `ur5e_toolheads` and restart the description/MoveIt stack. Compare TF and
collision geometry with the assembly before marking it calibrated. Configure
pendant TCP, payload and CoG separately. Create fresh paths and plans afterward.

## Syringe control

Flash `firmware/syringe_demo/syringe_demo.ino` on the Mega; uploading is separate
from Git updates. Close Serial Monitor. Configure a stable `/dev/serial/by-id/…`
port in a copied YAML if needed:

```bash
ls -l /dev/serial/by-id/
ros2 launch syringe_controller syringe.launch.py
# Or use an edited copy of src/syringe_controller/config/syringe.yaml:
ros2 launch syringe_controller syringe.launch.py config:="$HOME/syringe.yaml"
ros2 topic echo /syringe/status
ros2 service call /syringe/stop std_srvs/srv/Trigger '{}'
```

Bounded commands **move the pump**. Verify direction, remaining travel and a clear
nozzle first. Current calibration is 555.555556 pulses/mL, 203.006329 pulses/mm,
with extrusion direction -1. Counts and volumes are estimates, not sensor readings.

```bash
ros2 action send_goal /syringe/move_plunger syringe_interfaces/action/MovePlunger \
  '{steps: -20, step_rate_hz: 10.0}' --feedback
ros2 action send_goal /syringe/dispense_volume syringe_interfaces/action/DispenseVolume \
  '{volume_ml: 0.1, flow_ml_per_min: 1.0}' --feedback
# Negative volume retracts; verify available travel before use.
ros2 action send_goal /syringe/dispense_volume syringe_interfaces/action/DispenseVolume \
  '{volume_ml: -0.1, flow_ml_per_min: 1.0}' --feedback
```

Continuous flow **moves the pump** and needs a 2-second renewable lease. Prefer
finite volume actions for manual checks. Start a keepalive publisher only while
intentionally supervising flow; stop it after STOP:

```bash
ros2 topic pub --rate 5 /syringe/flow_keepalive std_msgs/msg/Empty '{}'
# Another terminal:
ros2 service call /syringe/set_flow syringe_interfaces/srv/SetFlow \
  '{enabled: true, flow_ml_per_min: 1.0, retract: false}'
ros2 service call /syringe/set_flow syringe_interfaces/srv/SetFlow \
  '{enabled: false, flow_ml_per_min: 0.0, retract: false}'
```

Low-level pulse flow uses the same lease and exclusive backend:

```bash
ros2 service call /syringe/set_extrusion syringe_interfaces/srv/SetExtrusion \
  '{enabled: true, direction: -1, step_rate_hz: 10.0}'
ros2 service call /syringe/stop std_srvs/srv/Trigger '{}'
```

Finite host goals are capped at 100 s; firmware commands at 3000 pulses/120 s.
Defaults include 0.2 s host PINGs and a 1.5 s serial-heartbeat deadline. Faults
latch: inspect the cause, reset the Mega and restart ROS. There is no automatic
homing/recovery. Complete manual serial command table, including `WATCHDOG OFF`,
`RAMP_STOP`, `STATUS`, `ESTOP`: [SYRINGE_FIRMWARE.md](SYRINGE_FIRMWARE.md).
Never open Serial Monitor while ROS owns the port.

## Capture, plan and check a small line

Captures the current nozzle pose, writes a world-meter CSV, and commands no motion:

```bash
ros2 run surface_printing make_demo_path --ros-args \
  -p shape:=line -p size_mm:=5.0 -p plane:=xy -p direction:=-1.0 \
  -p frame_id:=world -p link_name:=probe_tcp -p output_path:=/tmp/demo_path.csv
ros2 run surface_printing plan_surface_path --ros-args \
  -p csv_path:=/tmp/demo_path.csv -p frame_id:=world -p link_name:=probe_tcp \
  -p line_id:=0 -p cartesian_speed_mm_s:=1.0 \
  -p velocity_scale:=0.01 -p acceleration_scale:=0.01 \
  -p output_path:=/tmp/surface_line_plan.yaml
```

`shape` also accepts `circle` or `square`; `plane` accepts `xy`, `xz`, `yz`;
`direction` is ±1. Changing the shape recaptures a path from the live nozzle pose.
The Python planner selects exactly one line and does not insert an OMPL approach.
Start at the path's beginning with the pump stopped. Require a complete Cartesian
plan and inspect it in RViz. It checks joint continuity, wrist limits and sampled
TCP speed, uniformly slowing the saved trajectory if needed.

Check-only physical mode (default: no robot/pump motion):

```bash
ros2 run surface_printing execute_surface_print --ros-args \
  -p trajectory_path:=/tmp/surface_line_plan.yaml -p execute:=false
```

For a mock stack, use `-p simulation:=true` even for check-only:

```bash
ros2 run surface_printing execute_surface_print --ros-args \
  -p trajectory_path:=/tmp/surface_line_plan.yaml -p simulation:=true -p execute:=false
```

## Rehearse and print

**Mock motion**, no extrusion (requires verified live mock model):

```bash
ros2 run surface_printing execute_surface_print --ros-args \
  -p simulation:=true -p execute:=true -p extrude:=false
```

**Physical robot motion**, pump stopped (requires preflight/checklist and running
syringe STOP service, even for motion-only rehearsal):

```bash
ros2 run surface_printing execute_surface_print --ros-args \
  -p simulation:=false -p execute:=true -p extrude:=false -p hardware_confirmed:=true
```

Return manually to the same verified start with the pump stopped and **replan the
same CSV** after a rehearsal. Do not regenerate at the endpoint unintentionally.
Physical printing commands both robot and pump:

```bash
ros2 run surface_printing execute_surface_print --ros-args \
  -p simulation:=false -p execute:=true -p extrude:=true -p hardware_confirmed:=true \
  -p flow_ml_per_min:=1.0 -p max_duration_sec:=60.0 -p max_volume_ml:=1.0
```

Flow is a trial setting requiring material validation. Saved plans expire after
five minutes and bind to the live URDF/SRDF hash and starting joints. Scene,
model, placement or start changes require a new plan. Hard demo ceilings are
5 mL/min, 100 s and 5 mL. Budgets account for speed scaling and a 3 s reserve;
keep the pendant speed slider fixed. No automatic prime, final retract, retreat
or return to start is performed. For non-default output filenames also set
`trajectory_path` on every check/replay command.

## Tube helix from a manually positioned nozzle

Orient the tip within 15° of vertically downward, then position it at the first
circle-rim point. Mock capture bypasses physical calibration only after verifying
the live mock model and geometry:

```bash
ros2 run surface_printing make_tube_path --ros-args \
  -p simulation:=true -p diameter_mm:=10.0 -p height_mm:=1.0 \
  -p pitch_mm:=0.5 -p step_mm:=0.25 -p output_path:=/tmp/tube_10mm.csv
ros2 run surface_printing plan_surface_path --ros-args \
  -p csv_path:=/tmp/tube_10mm.csv -p frame_id:=world -p link_name:=probe_tcp \
  -p line_id:=0 -p output_path:=/tmp/tube_plan.yaml \
  -p cartesian_speed_mm_s:=2.0 -p velocity_scale:=0.03 \
  -p acceleration_scale:=0.03 -p eef_step:=0.0005
ros2 run surface_printing execute_surface_print --ros-args \
  -p trajectory_path:=/tmp/tube_plan.yaml -p simulation:=true -p execute:=false
```

Physical capture uses `simulation:=false` and requires calibrated real hardware.
The circle center is radius toward world -X from the captured tip, and the helix
rises along world +Z. Orientation stays fixed. Diameter is TCP path diameter,
not finished lumen diameter. No bead compensation or material support model is
included. Use the same check/rehearse/print sequence with `trajectory_path` set to
the tube plan. See [TUBE_DEMO.md](../TUBE_DEMO.md) for configuration and timing limits.

## Import an offline raster: choose one transform route

**Python route:** original object-frame millimeter CSV → registered world-meter
CSV → `plan_surface_path`. Apply the measured object-to-world transform once:

```bash
ros2 run surface_printing convert_surface_csv \
  "$HOME/Surface_printing_path_generation_pipeline/output/object_raster.csv" \
  /tmp/registered_raster.csv --units mm --xyz 0.55 -0.25 0.10 --rpy-deg 0 0 0
ros2 run surface_printing plan_surface_path --ros-args \
  -p csv_path:=/tmp/registered_raster.csv -p frame_id:=world -p link_name:=probe_tcp \
  -p line_id:=0 -p cartesian_speed_mm_s:=2.0
```

Those coordinates are syntax examples, not a current registration. The converter
transforms quaternion orientation too and preserves line IDs. It does not find the
object. Use the normalized mesh frame when measuring its pose.

**C++ route:** consume the original millimeter CSV, supplying its object pose.
Do not feed the converted meter CSV or apply the transform twice. This mock-only
example uses the installed saddle dataset and selects all lines:

```bash
ros2 run ur5e_moveit_cpp plan_execute_surface_toolpath --ros-args \
  -p csv_path:="$(ros2 pkg prefix --share ur5e_moveit_cpp)/data/toolpaths/saddle/raster_along_x.csv" \
  -p planning_group:=ur_manipulator -p tcp_link:=probe_tcp -p planning_frame:=world \
  -p object_x:=0.55 -p object_y:=-0.25 -p object_z:=0.10 \
  -p object_roll_deg:=0.0 -p object_pitch_deg:=0.0 -p object_yaw_deg:=0.0 \
  -p line_id:=-1 -p simulation:=true -p execute:=false -p extrude:=false \
  -p velocity_scale:=0.03 -p acceleration_scale:=0.03 -p eef_step:=0.002
```

Run plan-only first. For mock execution add `execute:=true` and
`confirm_mock_hardware:=true`. Each line has an OMPL approach and Cartesian trace.
The C++ physical mode additionally requires `simulation:=false`,
`hardware_confirmed:=true`, plus `extrusion_confirmed:=true` when extruding. It
has per-line/total flow budgets and stop-before-transition logic. Do not infer the
Python saved-plan TCP speed guarantee applies to this separate C++ executor;
verify registration and its speed on hardware independently. The Python bounded
workflow is the first-hardware route.

## Planning scene and raster inspection

Scene tools change collision objects but do not localize them. Example table box
(dimensions in meters, placeholder position):

```bash
ros2 run ur5e_moveit_cpp parameterized_shape_adder --ros-args \
  -p object_id:=table -p shape_type:=box -p frame_id:=world \
  -p center_x:=0.55 -p center_y:=0.0 -p center_z:=-0.05 \
  -p box_x:=1.0 -p box_y:=1.0 -p box_z:=0.1
```

`shape_type` also accepts `cylinder` (`radius`, `height`) and `sphere` (`radius`).
Add the saddle mesh with the **same measured pose** as its raster:

```bash
ros2 run ur5e_moveit_cpp add_mesh_collision_object --ros-args \
  -p mesh_resource:=package://ur5e_moveit_cpp/meshes/saddle/saddle_block_100x100.stl \
  -p object_id:=saddle -p frame_id:=world -p mesh_scale:=0.001 \
  -p object_x:=0.55 -p object_y:=-0.25 -p object_z:=0.10
```

`object_*` pose parameters use meters and degrees. Match them across mesh,
visualization, IK and planning commands.

Read-only raster/IK/planning tools (supply CSV and consistent object pose):

```bash
ros2 run ur5e_moveit_cpp visualize_surface_toolpath --ros-args -p object_x:=0.55
ros2 run ur5e_moveit_cpp check_surface_toolpath_ik --ros-args -p object_x:=0.55
ros2 run ur5e_moveit_cpp plan_surface_toolpath --ros-args -p line_id:=0
```

These defaults may select the front raster rather than the saddle; explicitly set
`csv_path` and object pose for your job. Visualization markers do not add collision
geometry. `remove_all_objects` clears **all** scene collision objects; use only
when rebuilding a mock scene, and re-add the table/workpiece before planning.

## RealSense processing

Optional camera launch and separate processing terminals:

```bash
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true align_depth.enable:=true
ros2 run ur5e_perception pointcloud_preprocessor
ros2 run ur5e_perception table_object_segmenter
ros2 run ur5e_perception object_measurement
ros2 topic echo /perception/object_center --once
ros2 topic hz /perception/object_cloud
```

Default input is `/camera/camera/depth/color/points`; inspect `ros2 topic list`
and override `input_topic` if your camera uses another namespace. Output chain:
`/perception/cleaned_cloud` → `/perception/object_cloud` → centroid and bounding
box. Filtering distances use meters in the incoming cloud frame. A centroid is
not a six-degree-of-freedom mesh registration or calibrated world pose.

## Legacy executable inventory

All are retained for compatibility. Inspect their source before lab use.

| `ur5e_moveit_cpp` executable | Effect |
|---|---|
| `simple_joint_motion`, `simple_pose_motion` | Executes hard-coded example motion after planning |
| `compare_pose_candidates`, `collision_aware_candidate_planner` | Chooses and executes a candidate |
| `collision_object_demo` | Changes scene and may execute motion |
| `hemisphere_surface_demo` | Plans; `execute` defaults false, true executes approach/path |
| `plan_only_trajectory`, `save_trajectory_csv`, `analyze_trajectory` | Planning/export/analysis experiments; inspect hard-coded paths/targets |
| `add_box_only`, `parameterized_shape_adder`, `add_mesh_collision_object` | Modifies collision scene |
| `remove_all_objects` | Clears the scene |
| `visualize_surface_toolpath` | Publishes path/mesh display markers |
| `check_surface_toolpath_ik`, `plan_surface_toolpath` | Checks IK / plans without execution |
| `plan_execute_surface_toolpath` | Plans by default; explicit execution mode can move robot/pump |

`paths/generate_hemisphere_ring.py` is a standalone generated-data utility;
review its output paths before running `python3 paths/generate_hemisphere_ring.py`.
Additional launch files in `ur5e_toolheads` and `ur5e_probe_moveit_config` are
component/debug launches; inspect their arguments with `ros2 launch PACKAGE FILE
--show-args`. Use `demo.launch.py` or `real_robot.launch.py` for the full stack.

## Launch-file inventory

Use `ros2 launch PACKAGE FILE --show-args` to inspect arguments before running.
Component launches are for debugging; avoid duplicate state publishers/controllers.

| Package / launch file | Role |
|---|---|
| `ur5e_toolheads view_probe.launch.py` | Static tool/arm visualizer with joint GUI and RViz; no hardware driver |
| `ur5e_toolheads ur5e_with_probe_real_rsp.launch.py` | Custom real description publisher used by real bring-up |
| `ur5e_probe_moveit_config demo.launch.py` | Full mock demo |
| `ur5e_probe_moveit_config real_robot.launch.py` | Full real stack with mandatory kinematics calibration |
| `ur5e_probe_moveit_config move_group.launch.py`, `move_group_real.launch.py` | Mock / physical MoveIt server components |
| `ur5e_probe_moveit_config moveit_rviz.launch.py`, `moveit_rviz_real.launch.py` | Mock / physical RViz components |
| `ur5e_probe_moveit_config rsp.launch.py`, `static_virtual_joint_tfs.launch.py` | Description / virtual-joint TF components |
| `ur5e_probe_moveit_config spawn_controllers.launch.py` | Mock controller spawning |
| `ur5e_probe_moveit_config warehouse_db.launch.py` | MoveIt warehouse component |
| `ur5e_probe_moveit_config setup_assistant.launch.py` | MoveIt configuration editing GUI |
| `syringe_controller syringe.launch.py` | Serial pump controller, optional `config` YAML argument |

## Stop and recover

While the Python executor is running, in another terminal on the same ROS domain:

```bash
ros2 service call /surface_printing/stop std_srvs/srv/Trigger '{}'
ros2 service call /syringe/stop std_srvs/srv/Trigger '{}'
```

The surface stop service exists only while that executor process is alive; it is
not the stop endpoint for legacy C++ demos. Pump STOP alone does not stop the arm.
Use the pendant/physical robot stop and independent pump power cut for hazardous
motion. Ctrl-C requests cleanup in the Python executor. A service reply is not
proof of physical cessation or pressure relief. After faults, clear the cause,
restart as required, reposition safely and create a fresh plan; never auto-resume.

## Validation

Offline checks from the repository root:

```bash
python3 -m pip install numpy pyyaml
python3 -m unittest discover -s tests -v
python3 -m compileall -q src/surface_printing src/syringe_controller src/ur5e_toolheads
g++ -std=c++17 -I tests/firmware_stub tests/test_firmware.cpp -o /tmp/test_syringe_firmware
/tmp/test_syringe_firmware
```

On the ROS host, build and check integration before robot motion:

```bash
colcon build --symlink-install
colcon test
colcon test-result --verbose
ros2 pkg executables ur5e_moveit_cpp
ros2 launch ur5e_probe_moveit_config real_robot.launch.py --show-args
```

Offline tests do not exercise ROS transport, MoveIt planning, AVR timing, hardware
clearance, actual flow, stop latency or complete physical execution.
