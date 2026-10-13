# Lead-in and circle printing

Generate a fresh path from the current nozzle pose on every run. The generator
commands no robot or pump motion. Defaults: 10 mm straight lead-in toward
world -X, 2 mm target inner diameter, 0.5 mm measured bead width, one circle,
0.1 mm sample spacing, fixed XY plane, height and nozzle orientation.
The path diameter is inner diameter plus bead width (2.5 mm by default).
The nominal opening depends on actual deposition; remeasure bead width when
changing material, extrusion frequency, nozzle height or travel speed.
The full default path extends 12.5 mm toward -X from the captured pose.

In each terminal, source ROS and the workspace and set ROS_DOMAIN_ID=77.
Keep the physical robot/MoveIt and syringe launches running.

## Generate

```bash
ros2 run surface_printing make_lead_circle_path --ros-args \
  -p lead_in_mm:=10.0 -p inner_diameter_mm:=2.0 -p bead_width_mm:=0.5
```

Optional parameters: spacing_mm (default 0.1), direction (-1.0 or 1.0),
passes (positive integer up to 100), frame_id (world), link_name (probe_tcp),
output_path (/tmp/line_circle.csv). All circle turns follow the lead-in in
one line_id, with no duplicated junction waypoint.

## Plan

```bash
ros2 run surface_printing plan_surface_path --ros-args \
  -p csv_path:=/tmp/line_circle.csv \
  -p frame_id:=world -p link_name:=probe_tcp \
  -p cartesian_speed_mm_s:=6.0 \
  -p velocity_scale:=0.03 -p acceleration_scale:=0.03 \
  -p eef_step:=0.0001 -p output_path:=/tmp/line_circle_plan.yaml
```

Inspect the successful plan in RViz. A planning failure does not replace any
previous saved plan; never execute a stale file after a failed planning attempt.
If the nozzle moved after generation, regenerate. Joint limits can reduce actual
speed below the requested Cartesian ceiling.

## Execute

After successful preflight and reviewing clearance:

```bash
ros2 run surface_printing execute_surface_print --ros-args \
  -p trajectory_path:=/tmp/line_circle_plan.yaml \
  -p simulation:=false -p execute:=true -p extrude:=true \
  -p step_rate_hz:=200.0 -p extrusion_direction:=-1 \
  -p max_pulses:=0 -p max_duration_sec:=900.0 \
  -p start_tolerance_rad:=0.005 -p hardware_confirmed:=true
```

Extrusion starts after measured robot motion and continues through the junction.
The executor stops the pump at completion or fault. No automatic return, retract
or approach is commanded. Keep the pendant speed slider fixed during execution.
