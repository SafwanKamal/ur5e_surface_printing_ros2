# Tube printed from adjacent vertical columns

## Motion pattern

`make_column_tube_path` captures the current nozzle pose, prints a straight
world -X lead-in, then vertical strands around the tube circumference.
Column 0 goes bottom-to-top; a short circumferential arc at the top leads to
column 1, which goes top-to-bottom. It repeats this serpentine pattern around
the cylinder. An even count lets the last column end at the base, followed by
a short base arc closing the seam. All segments share line_id=0.
Extrusion remains on during the lead, vertical strokes and connectors.
This is a vertical-column wall, with short extruded connections at its ends.

This implementation is intended for a supported-material experiment. It does
not implement upward-only strokes with pump-off travel between columns.
The nozzle stays downward with the captured orientation. Generator checks
for the same live model, calibrated tool, fresh joints and downward orientation
as the helix generator. It does not command robot or pump motion.

## Parameters

| Parameter | Default | Meaning |
|---|---:|---|
| lead_in_mm | 10.0 | Straight lead-in toward world -X |
| inner_diameter_mm | 10.0 | Target opening, approximate after compensation |
| bead_width_mm | 0.5 | Measured lateral strand width under actual vertical printing conditions |
| height_mm | 3.0 | Vertical stroke length |
| overlap_fraction | 0.2 | Desired fractional overlap between adjacent strand widths |
| column_count | 0 | Automatic; override with an even integer >=4 |
| step_mm | 0.1 | Maximum nominal sample spacing |
| simulation | false | Validate physical hardware model by default |
| output_path | /tmp/column_tube.csv | CSV consumed by existing planner |

Path diameter = inner_diameter_mm + bead_width_mm. This assumes a symmetric
bead about the centerline and is only an approximate opening correction.
For automatic spacing, desired center spacing = bead_width_mm *
(1-overlap_fraction). The smallest even number N >=4 satisfying
path_diameter_mm * sin(pi/N) <= desired center spacing is selected.
At default settings N=84, adjacent chord spacing is about 0.393 mm,
and nominal path length is about 295 mm. The full path extends 20.5 mm
toward world -X and 3 mm upward from the captured nozzle tip.

CSV phase and column_index metadata distinguish lead-in, columns and connectors.
The existing planner ignores those extra columns while using all poses.
An explicit column_count with center spacing exceeding bead width emits a warning.
Measured overlap is not proof that the wall has fused or is leak-tight.

## Install and build

```bash
cd ~/ur5e_ws && git pull --ff-only origin main && \
source /opt/ros/jazzy/setup.bash && \
colcon build --symlink-install --packages-select surface_printing && \
source install/setup.bash
export ROS_DOMAIN_ID=77
```

Keep the real driver/MoveIt and syringe nodes running in domain 77.

## 1. Generate from current position

```bash
ROS_DOMAIN_ID=77 ros2 run surface_printing make_column_tube_path --ros-args \
  -p lead_in_mm:=10.0 -p inner_diameter_mm:=10.0 \
  -p bead_width_mm:=0.5 -p height_mm:=3.0 -p overlap_fraction:=0.2
```

## 2. Plan

```bash
ROS_DOMAIN_ID=77 ros2 run surface_printing plan_surface_path --ros-args \
  -p csv_path:=/tmp/column_tube.csv -p frame_id:=world -p link_name:=probe_tcp \
  -p cartesian_speed_mm_s:=1.0 -p velocity_scale:=0.03 \
  -p acceleration_scale:=0.03 -p eef_step:=0.0001 \
  -p output_path:=/tmp/column_tube_plan.yaml
```

Inspect the full successful plan in RViz. Regenerate if the nozzle moves.
No failed plan should be followed by execution of a previous saved file.

## 3. Execute

After passing hardware_preflight and verifying clearance:

```bash
ROS_DOMAIN_ID=77 ros2 run surface_printing execute_surface_print --ros-args \
  -p trajectory_path:=/tmp/column_tube_plan.yaml \
  -p simulation:=false -p execute:=true -p extrude:=true \
  -p step_rate_hz:=275.0 -p extrusion_direction:=-1 \
  -p max_pulses:=0 -p max_duration_sec:=900.0 \
  -p start_tolerance_rad:=0.005 -p hardware_confirmed:=true
```

Actual duration can exceed length/speed due to retiming, joint limits,
corners and pendant speed scaling. The existing configurable run duration
applies; this generator adds no separate elapsed-time cutoff.

## Feasibility and experimental checks

- Robot motion: straightforward Cartesian translation with fixed orientation,
  but full IK/collision/timing validation is still required for each live pose.
- Support: vertical soft/liquid strands need adequate support or sufficiently
  fast solidification. Embedded support baths can hold printed material.
- Up/down asymmetry: axial motion is different from lateral helix printing.
  Calibrate upward and downward strands separately; do not assume a bead width
  measured from a horizontal line is the correct vertical-filament diameter.
- Nozzle contact: during descending strokes, the needle shaft can run beside
  earlier full-height strands. Check the exposed needle length, outer diameter,
  taper and holder clearance. The planner does not automatically model the wall
  as it is being printed.
- Fusion: adjacent strands can remain separated by support material or fail
  to bond. Inspect a cured cross-section and test leak-tightness.
- End connections: sharp changes between vertical strokes and small arcs can
  slow the robot, increasing local deposition with constant pump frequency.
  Check for thick end rims or blobs.
- In-air printing: this code is not evidence of freestanding printability.
  A short supported test is appropriate before increasing height.

Research basis: Bhattacharjee et al., Writing in the granular gel medium,
Science Advances 1:e1500655 (2015), https://doi.org/10.1126/sciadv.1500655;
Ren et al., Study of sacrificial ink-assisted embedded printing for 3D perfusable
channel creation for biomedical applications (2022),
https://pmc.ncbi.nlm.nih.gov/articles/PMC8785228/.

Offline tests verify geometry, column spacing, exact base closure,
vertical up/down strokes, fixed orientation, live-start capture and rejection
of an upward-facing nozzle. ROS transport, real robot motion and materials
have not been tested in the development environment.
