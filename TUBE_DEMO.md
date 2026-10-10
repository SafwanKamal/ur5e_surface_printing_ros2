# Nominal 10 mm tube from a manually positioned nozzle

This adds a continuous helix in world XY rising along world +Z. The captured tip
is the first printing point on the circumference, not the center. The center is
5 mm toward world -X from the captured tip. Every waypoint retains the captured
orientation. No automatic wrist rotation is performed. Position the nozzle nearly
vertically downward (within 15 degrees) before capturing. Confirm the entire
10 mm circle and upward growth direction are physically clear.

`diameter_mm=10` is nominal TCP path diameter, not a certified 10 mm finished
inner diameter. No bead-width compensation is included. Height must be supplied;
the default 0.5 mm pitch is configurable. Continuous extrusion on a helix is not
proof that your current liquid will support a freestanding tube.

## Pull and build

Stop active robot/pump execution first. These commands preserve your local YAML.
This update does not edit `printing_tool.yaml` or auto-mark it calibrated.

```bash
cd "$HOME/ur5e_ws" && git pull --ff-only origin demo/coarse-collision
```

```bash
cd "$HOME/ur5e_ws" && source /opt/ros/jazzy/setup.bash && export CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS="-j1" && colcon build --symlink-install --executor sequential --packages-select ur5e_toolheads surface_printing --allow-overriding ur5e_toolheads surface_printing && source install/setup.bash
```

## Configure the new nozzle

160 mm is TOTAL flange-face-to-tip length, not a needle extension added to the
old CAD outlet. UR's `tool0` +Z is the physical outward axis; the ROS `flange`
frame uses a different orientation. The resulting `tool0 -> probe_tcp` must be
`xyz=[0,0,0.160]`, identity rotation.

Enter the measured maximum OUTER nozzle radius (in mm); this is required for its
collision cylinder. Do not use bore/inner radius. The old pump body CAD geometry
and collisions remain unchanged and need comparison against the actual assembly.
If the pump mounting also changed, its model requires a separate correction.

```bash
source /opt/ros/jazzy/setup.bash && source "$HOME/ur5e_ws/install/setup.bash" && export ROS_DOMAIN_ID=77 && read -r -p 'Maximum nozzle outer radius (mm): ' NOZZLE_RADIUS_MM && ros2 run ur5e_toolheads set_axial_nozzle_tcp --config "$HOME/ur5e_ws/src/ur5e_toolheads/config/printing_tool.yaml" --length-mm 160 --radius-mm "$NOZZLE_RADIUS_MM" --write
```

Stop and restart your existing physical robot/MoveIt launch after rebuilding the
toolhead package. Use your actual extracted `kinematics_params_file`; do not
switch to a default calibration or a mock stack. RViz must show the nozzle and
tip matching the hardware. Check the transform in a sourced terminal:

```bash
source /opt/ros/jazzy/setup.bash && source "$HOME/ur5e_ws/install/setup.bash" && export ROS_DOMAIN_ID=77 && timeout 10 ros2 run tf2_ros tf2_echo tool0 probe_tcp
```

Only after verifying the physical tip position, orientation, pump model and
nozzle collision radius, manually change `calibrated: false` to
`calibrated: true` in source `printing_tool.yaml`. Rebuild/restart as needed and
run preflight. Do not remove collision objects or disable checking to pass it.

```bash
source /opt/ros/jazzy/setup.bash && source "$HOME/ur5e_ws/install/setup.bash" && export ROS_DOMAIN_ID=77 && ros2 run surface_printing hardware_preflight
```

## Capture and plan (no motion or extrusion)

For mock-only tube capture, run the demo stack and all preview commands in
`ROS_DOMAIN_ID=78`, position the mock nozzle downward, and add
`-p simulation:=true` to `make_tube_path`. This checks the live mock hardware
plugin and all configured tool geometry without requiring `calibrated: true`.
Physical capture defaults to `simulation:=false` and still requires the live UR
driver model and verified physical calibration. Never relabel a tool calibrated
solely to generate a simulation path.

Manually place the tip on the intended circle rim at the first-layer height.
Orient it downward first; then position the tip. Capture once. Use the same CSV
for preview and execution. Coordinates are already in world meters for the
Python planner. Do not apply the old saddle position or tool-roll search.

```bash
source /opt/ros/jazzy/setup.bash && source "$HOME/ur5e_ws/install/setup.bash" && export ROS_DOMAIN_ID=77 && read -r -p 'Tube height (mm): ' TUBE_HEIGHT_MM && ros2 run surface_printing make_tube_path --ros-args -p diameter_mm:=10.0 -p height_mm:="$TUBE_HEIGHT_MM" -p pitch_mm:=0.5 -p step_mm:=0.25 -p output_path:=/tmp/tube_10mm.csv
```

```bash
source /opt/ros/jazzy/setup.bash && source "$HOME/ur5e_ws/install/setup.bash" && export ROS_DOMAIN_ID=77 && ros2 run surface_printing plan_surface_path --ros-args -p csv_path:=/tmp/tube_10mm.csv -p frame_id:=world -p link_name:=probe_tcp -p group_name:=ur_manipulator -p line_id:=0 -p output_path:=/tmp/tube_plan.yaml -p cartesian_speed_mm_s:=2.0 -p velocity_scale:=0.03 -p acceleration_scale:=0.03 -p eef_step:=0.0005
```

Check RViz: complete helix, starting from your current pose, with no OMPL
approach or arbitrary IK branch transition. The planner requires full completion
and validates joint continuity and sampled TCP speed. Your real floor, supports, bath/container
and fixtures must be represented or verified clear. Do not change the speed
slider mid-print. If robot placement/model/scene changes, capture/plan again.

## Execute with extrusion (explicitly commands hardware)

The executor uses the saved, previewed plan; it does not replan or command an
approach. It rejects changed models, stale plans (over five minutes), changed
start joints or invalid current collisions. Syringe flow starts after measured
robot progress and stops at completion/fault. Existing watchdog and volume/time
limits remain active. There is no automatic final retract.
Choose a low verified flow for the actual liquid. Watch the first turn
and stop if material pools, the tip drags, or the structure cannot support itself.

```bash
source /opt/ros/jazzy/setup.bash && source "$HOME/ur5e_ws/install/setup.bash" && export ROS_DOMAIN_ID=77 && read -r -p 'Verified extrusion flow (ml/min): ' TUBE_FLOW && ros2 run surface_printing hardware_preflight && ros2 run surface_printing execute_surface_print --ros-args -p trajectory_path:=/tmp/tube_plan.yaml -p simulation:=false -p execute:=true -p extrude:=true -p flow_ml_per_min:="$TUBE_FLOW" -p max_duration_sec:=300.0 -p max_volume_ml:=2.0 -p start_tolerance_rad:=0.005 -p hardware_confirmed:=true
```

The limits may reject a tall tube; do not disable them blindly. Start short and
review planned duration and extrusion volume before increasing limits.

Pump stop in another sourced terminal (does not stop robot motion):

```bash
source /opt/ros/jazzy/setup.bash && source "$HOME/ur5e_ws/install/setup.bash" && export ROS_DOMAIN_ID=77 && ros2 service call /syringe/stop std_srvs/srv/Trigger '{}'
```

Use the pendant's stop/emergency stop for hazardous robot motion.

## Validation performed

Offline tests cover helix radius/height/start/spacing, invalid dimensions,
quaternion normalization and TCP composition including nonzero mounting offsets
and rotation singularities. No ROS 2 runtime, live robot or syringe was available
in the development environment. Build and integration verification are required
on the host PC before hardware execution.
