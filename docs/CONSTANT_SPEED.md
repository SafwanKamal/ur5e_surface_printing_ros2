# Consistent Cartesian deposition speed

The planner now has two explicit speed parameters:

- max_cartesian_speed_mm_s: maximum sampled translational speed at the chosen tip.
- average_cartesian_speed_mm_s: requested steady deposition speed. The percentage
  of traveled distance within +/-5% is reported, not used as an admission limit.

The second parameter is a local cruising-speed target, not a promise that total
distance divided by total elapsed time equals the target. The planner reports
both target-band coverage and the actual overall planned average. Starts, stops
and necessary turns are included in the coverage denominator.

For existing commands, cartesian_speed_mm_s remains an alias supplying the
default maximum. If the new average parameter is omitted, it defaults to the
maximum. New plans use the consistent-speed retimer. Necessary slowdowns are
allowed even when target-band coverage is below 90%; the achieved planned
average and coverage remain visible in the log and saved metadata.

## Timing algorithm

The validated IK waypoint positions are retained. Timestamped MoveIt timing is
replaced with local Cartesian-distance timing. Monotone cubic joint derivatives
produce continuous velocities, with zero velocity at start/end and sharp turns
of 60 degrees or more. Only intervals that violate sampled TCP maximum or exact
cubic joint velocity/acceleration bounds are lengthened. Neighboring derivatives
are recomputed as interval durations change.

This preserves configured velocity_scale and acceleration_scale, the existing
0.3 rad/s joint ceiling and Wrist 3 bounds. It does not defeat hardware limits
or move corners off the requested path to obtain higher coverage.
A 0.1% numerical margin is used for sampled maximum speed, rather than the old
global 5% speed reduction. Final spline validation uses 101 samples per interval.
Joint velocity and acceleration extrema are checked analytically for each cubic.

The target-band percentage is integrated by traveled TCP distance using the
sampled Jacobian speed, not by waypoint count or time. There is no minimum
target-band coverage requirement; a valid slower plan can be saved and executed.
The previous output file is removed before attempting the new speed plan to
avoid accidentally executing old timing after a failure.

## Example: helix at nominal 1 mm/s

Generate from the current pose first, then:

```bash
ROS_DOMAIN_ID=77 ros2 run surface_printing plan_surface_path --ros-args \
  -p csv_path:=/tmp/line_helix.csv \
  -p frame_id:=world -p link_name:=probe_tcp \
  -p max_cartesian_speed_mm_s:=1.2 \
  -p average_cartesian_speed_mm_s:=1.0 \
  -p velocity_scale:=0.03 -p acceleration_scale:=0.03 \
  -p eef_step:=0.0001 -p output_path:=/tmp/line_helix_plan.yaml
```

The 1.2 maximum is allowable headroom; it is not the cruising target.
Use maximum=1.0 and average=1.0 if the peak must remain at 1 mm/s.
If the requested target cannot be maintained under the joint limits, intervals
are slowed and the achieved speed is reported. To apply to columns, use
/tmp/column_tube.csv and /tmp/column_tube_plan.yaml. Sharp short connectors can
reduce target-band coverage compared with a smooth helix.

## Execution and measured feedback

The executor independently recomputes the speed coverage and scaled joint
limits from the saved trajectory and current model before sending motion.
For target-speed plans, effective UR scaling must be within 1% of 100% at
startup; keep the pendant slider fixed throughout the print.

The executor estimates actual tip translation from the current joint positions
and live calibrated URDF using roughly 0.25-second timestamped windows.
It logs current measured tip speed and accumulated target-band coverage every
five seconds, then reports measured average, peak and coverage at completion.

Measured speed is report-only. Neither overspeed readings nor low measured
target-band coverage cancel execution or turn successful completion into failure.
The executor logs average, peak and coverage and emits summary warnings if needed.
The planner and the executor's pre-motion trajectory validation still enforce
the maximum TCP speed and scaled joint limits on the commanded plan.
Commanded target-band coverage is also report-only, including during pre-motion
validation of previously saved plans.
Existing robot, syringe, keepalive, feedback and configured-duration checks
remain active. See [ABORT_RECOVERY.md](ABORT_RECOVERY.md) for stop verification.

Both spline sampling and finite-difference feedback are estimates, not a
continuous-time bound or safety-rated physical speed guarantee. Live speed
tracking depends on correct nozzle calibration and timestamped joint feedback.
Short windows near starts/stops and tight curves can differ from instantaneous
speed. The first hardware run must establish actual tracking quality; development
tests do not constitute physical robot verification.

Old plans without average-speed metadata retain their legacy validation.
Regenerate and replan to obtain the new timing and measured coverage behavior.

## Validation

Tests cover line and helix timing, nonuniform sample spacing, a revolute chain,
equal maximum/target settings, targets slowed to respect joint limits,
corner-heavy short paths and slowed timing accepted below 90% coverage,
maximum-speed and joint-limit violation rejection, rest endpoints, geometry preservation, vectorized Jacobian
agreement, report-only overspeed and low measured coverage, repeated timestamps
and clock reversal. Existing TCP speed, Cartesian timing and executor tests
are also run. No robot or ROS transport is available in the development container.
