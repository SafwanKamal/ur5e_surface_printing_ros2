# Lead-in and helix

`make_lead_helix_path` captures the current physical nozzle pose each run and
writes one continuous path: a straight world -X lead-in followed by a helix
in XY rising along world +Z. Orientation stays fixed. It never commands motion
or extrusion. The existing tube tool, FK, fresh-joint-state and downward-nozzle
checks remain in place; the nozzle must point downward within 15 degrees.

Defaults: lead_in_mm=10, inner_diameter_mm=2, bead_width_mm=0.5,
height_mm=10, pitch_mm=0.5, step_mm=0.1, simulation=false,
output_path=/tmp/line_helix.csv. Path diameter = inner diameter + bead width.
At default settings there are 20 turns, a 2.5 mm centerline diameter,
12.5 mm total world -X extent, and 10 mm upward growth.
Compensation assumes a measured symmetric bead; actual inner diameter depends
on material deposition and should be measured after changing printing settings.

Source ROS and the workspace; set ROS_DOMAIN_ID=77 in every physical terminal.
Keep robot/MoveIt and syringe launches running.

## Generate from current position

```bash
ros2 run surface_printing make_lead_helix_path --ros-args \
  -p lead_in_mm:=10.0 -p inner_diameter_mm:=2.0 -p bead_width_mm:=0.5 \
  -p height_mm:=10.0 -p pitch_mm:=0.5
```

## Plan

```bash
ros2 run surface_printing plan_surface_path --ros-args \
  -p csv_path:=/tmp/line_helix.csv -p frame_id:=world -p link_name:=probe_tcp \
  -p cartesian_speed_mm_s:=6.0 -p velocity_scale:=0.03 \
  -p acceleration_scale:=0.03 -p eef_step:=0.0001 \
  -p output_path:=/tmp/line_helix_plan.yaml
```

Inspect the complete successful plan in RViz. Regenerate if the nozzle moved.
A failed planning attempt does not replace the previous saved plan.
The scene and physical clearance must cover the lead-in and complete upward growth.

## Execute with extrusion

After successful hardware_preflight and inspecting the plan:

```bash
ros2 run surface_printing execute_surface_print --ros-args \
  -p trajectory_path:=/tmp/line_helix_plan.yaml \
  -p simulation:=false -p execute:=true -p extrude:=true \
  -p step_rate_hz:=200.0 -p extrusion_direction:=-1 \
  -p max_pulses:=0 -p max_duration_sec:=900.0 \
  -p start_tolerance_rad:=0.005 -p hardware_confirmed:=true
```

Extrusion stays enabled from the lead-in into the helix and stops on completion
or fault. No automatic approach, return or retract. Keep the pendant speed slider
fixed; joint limits may reduce travel speed below the requested ceiling.
Pitch is vertical rise per revolution, not a guaranteed deposited layer height.
