# Syringe path-printing demo

This branch supplies a bounded, constant-flow demo, not closed-loop material
control. Start with one 30 mm straight line. Default requested flow is 1 mL/min
(about 0.972 mL/min after integer pulse-rate rounding with the current calibration).
The Cartesian speed cap is 2 mm/s before UR speed scaling. Acceleration, corners,
pressure lag and the delay until measured motion starts can change bead width.
Do not present this as fully synchronized or validated conformal printing.

## Required physical setup

- Flash `firmware/syringe_demo/syringe_demo.ino` to the Mega 2560 at 115200 baud.
  Close the Arduino serial monitor before starting ROS. PUL+ pin 28, DIR+ pin 27,
  common signal ground as in the working lab wiring. ENA pin 25 is unused by
  default (`USE_ENABLE=false`); verify driver polarity before enabling it.
- Firmware optional NC interlocks use pins 30 (stop), 32 (extrusion end), 34
  (retraction end), healthy closed to ground. They are DISABLED by default and
  are not safety-rated. Enable `USE_INTERLOCKS` only after installing/testing all
  switches. Firmware faults latch until reset. Never reset to bypass an active trip.
- Verify extrusion direction, microstep settings, plunger contact and remaining
  stroke/volume. The configured -1 direction and 555.555556 pulses/mL are prior
  measurements, not proof of the current assembly. At first, test without pressure
  and with the nozzle away from the workpiece. Do not run against a blocked nozzle.
- Complete [pump alignment and TCP calibration](PUMP_MODEL.md). The full STL is
  included, but its mounting transform and nozzle TCP need physical measurement.
  Do not merely set `calibrated: true` to bypass the gate. Configure payload/CoG
  and pendant TCP separately. Check the whole moving plunger and tubing envelope.
- Provide an accessible robot emergency stop AND a way to cut syringe-driver
  power. A UR emergency stop does not directly cut this independent Arduino pump.
  ROS stop, USB watchdog and optional Arduino switches are software/electronic
  fallbacks, not a certified coupled emergency-stop circuit.
- Model the table, fixtures and workpiece in MoveIt. Verify nozzle standoff and
  clearance physically. No camera/localization is required for the planar demo,
  but it will not follow an unknown curved surface.

## Get and build

In your existing repository, first preserve any local changes. Do not use a hard
reset to resolve divergence. These commands assume it is `~/ur5e_ws/src`:

```bash
cd ~/ur5e_ws/src
 git fetch origin
 git switch demo/syringe-print
 git pull --ff-only origin demo/syringe-print
cd ~/ur5e_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select syringe_interfaces syringe_controller ur5e_toolheads ur5e_probe_moveit_config surface_printing
source install/setup.bash
```

If the branch is not yet local, `git switch --track origin/demo/syringe-print`
creates it. If an existing local demo branch has diverged, reconcile it before
building; do not discard lab changes. Edit the source `printing_tool.yaml`, rebuild
and restart all description/MoveIt launches after calibration changes.

## Start the stack (separate terminals)

Source ROS and `~/ur5e_ws/install/setup.bash` in each terminal. Use your actual IPs:

```bash
ros2 launch ur5e_probe_moveit_config real_robot.launch.py robot_ip:=192.168.1.102 reverse_ip:=192.168.1.100
```

Run the pendant External Control program using the existing lab setup. Confirm
`scaled_joint_trajectory_controller` is active and RViz matches the real robot.
Keep the speed slider fixed during each run. The executor reads the UR
`/speed_scaling_state_broadcaster/speed_scaling` topic. Installed UR driver
versions publish either a 0..1 factor or a 0..100 percentage; the executor
normalizes both and aborts on stale/zero feedback or a change over two points.

```bash
ros2 launch syringe_controller syringe.launch.py
ros2 service call /syringe/stop std_srvs/srv/Trigger '{}'
```

If needed, copy/edit `src/syringe_controller/config/syringe.yaml` and launch with
`config:=/absolute/path/syringe.yaml`. Use `/dev/serial/by-id/...` for a stable USB
port. Startup must report `READY SYRINGE_DEMO_V1`; old firmware is rejected.
Only one program may own the serial port or command the robot/pump during a demo.

## First pump-only check

With ample free travel and a clear nozzle over a collection cup, command a small
finite dispense. Watch the physical direction and be ready to stop:

```bash
ros2 action send_goal /syringe/dispense_volume syringe_interfaces/action/DispenseVolume "{volume_ml: 0.1, flow_ml_per_min: 1.0}" --feedback
```

That is about 56 pulses with the current calibration. Confirm observed movement,
then verify the stop command while dispensing. Do not adjust the sign based only
on ROS success: there is no encoder or flow sensor. Prime separately into the cup
using bounded dispense actions; no automatic prime or retract is included.

## Generate and plan (no motion)

Use the pendant to place the nozzle at a verified start/standoff with extrusion
OFF. The path starts at the present nozzle pose; the line extends +X in world.
Choose `direction:=-1.0` for -X, or another `plane` only after checking clearance.

```bash
ros2 run surface_printing make_demo_path --ros-args -p shape:=line -p size_mm:=30.0 -p plane:=xy -p frame_id:=world -p link_name:=probe_tcp
ros2 run surface_printing plan_surface_path --ros-args -p csv_path:=/tmp/demo_path.csv -p frame_id:=world -p link_name:=probe_tcp -p cartesian_speed_mm_s:=2.0 -p velocity_scale:=0.03 -p acceleration_scale:=0.03
ros2 run surface_printing execute_surface_print
```

The last command is CHECK ONLY by default. Inspect the complete path in RViz.
Plans expire after five minutes and bind to the live URDF/SRDF hash. Execution
rechecks sampled states against the current planning scene and robot start.
All planning must succeed; partial Cartesian paths are rejected. A missing
Cartesian speed field in `moveit_msgs` fails explicitly rather than ignoring it.
The collision recheck is sampled, not a continuous collision guarantee.

## Motion-only rehearsal, then printing

Rehearse with no material flow, after satisfying the physical checklist:

```bash
ros2 run surface_printing execute_surface_print --ros-args -p execute:=true -p extrude:=false -p hardware_confirmed:=true
```

After rehearsal, manually return to the verified starting pose with the pump
stopped. Re-run the planner for the SAME `/tmp/demo_path.csv` so it uses current
joint positions. Do not regenerate a new path at the line's endpoint unless you
intend to print in a different place. Recheck standoff, then print:

```bash
ros2 run surface_printing plan_surface_path --ros-args -p csv_path:=/tmp/demo_path.csv -p frame_id:=world -p link_name:=probe_tcp -p cartesian_speed_mm_s:=2.0
ros2 run surface_printing execute_surface_print --ros-args -p execute:=true -p extrude:=true -p hardware_confirmed:=true -p flow_ml_per_min:=1.0 -p max_duration_sec:=60.0 -p max_volume_ml:=1.0
```

These flow/time/volume settings are demonstration starting values, not material
process recommendations. Begin with a straight line; square corners can blob at
constant flow. `shape:=circle` and `shape:=square` are supported, but plan duration
can exceed the default budget. Reduce size or flow as appropriate; don't casually
raise limits. Executor hard ceilings: 5 mL/min, 100 s, 5 mL. Firmware additionally
limits each run to 3000 pulses / 120 s; available syringe stroke may be less.
At 50% UR scaling a nominal 15 s path needs about 30 s; the budget calculation
accounts for scaling and reserves three seconds. Actual rate is pulse-estimated.
There is no cumulative stroke tracking, force sensing or clog detection.

## Stop and recover

In a separate sourced terminal:

```bash
ros2 service call /surface_printing/stop std_srvs/srv/Trigger '{}'
ros2 service call /syringe/stop std_srvs/srv/Trigger '{}'
```

Ctrl-C also initiates bounded robot cancellation and pump STOP. A successful
`/surface_printing/stop` reply acknowledges the request, not physical cessation.
Watch for stop/cancel confirmation; if absent or motion continues, use the physical
stops. Never resume automatically after any fault. Inspect, clear the cause,
relaunch the controller if fault-latched, reposition safely and make a fresh plan.
The executor does not move away, retract, retry or return to start automatically.

Guards cover rejected/failed trajectories, delayed goal acceptance, stale joint
or speed feedback (0.5 s), no measured joint progress (1 s), stale/nonmoving syringe
status, time/estimated volume bounds, operator abort and model/start mismatch.
They do not prove TCP motion: joint movement could occur with little nozzle travel.
The pump lease expires after 2 s without keepalives; firmware stops after 1.5 s
without host PING. Detection and communication latency allow residual extrusion.
STOP cannot instantly relieve stored pressure. A delayed/lost flow-service reply
can leave uncertainty until STOP/lease/watchdog handling completes.

## Simulation and existing curved paths

For mock-only rehearsal, shut down physical robot drivers, disconnect pump power,
and use a separate ROS domain to avoid accidentally connecting to lab controllers:

```bash
export ROS_DOMAIN_ID=77
ros2 launch ur5e_probe_moveit_config demo.launch.py
```

Use the same domain in all mock terminals. Generate/plan as above, then run
`execute_surface_print --ros-args -p execute:=true -p simulation:=true`.
Simulation forbids extrusion and requires the live MoveIt model to use
`mock_components/GenericSystem`. This model check is not hardware isolation;
shutting down physical drivers and using the separate domain remain necessary.

To use the other repository's exported millimeter CSV, first measure its rigid
object-to-world transform. This converter is installed in `surface_printing`:

```bash
ros2 run surface_printing convert_surface_csv input.csv registered.csv --units mm --xyz 0.60 0.00 0.18 --rpy-deg 0 0 90
```

The numbers above are syntax examples, NOT a measured current registration.
Converter changes both position and quaternion, preserves `line_id`, and emits
meters. Plan one selected line with `-p line_id:=0`; position at its start with
extrusion off.

### Multi-line saddle execution

`plan_execute_surface_toolpath` can plan every line with `line_id:=-1`. Each
line is represented by a collision-aware OMPL transition followed by a Cartesian
surface trace. Physical extrusion is gated per trace: STOP is acknowledged before
every transition, measured robot motion must begin before flow starts, and STOP is
acknowledged after every trace. A speed-scaling change, stale joint/syringe feedback,
one second without joint progress, a pump fault, a duration/volume overrun or a
trajectory failure cancels the active robot goal and requests pump STOP.

Always run `hardware_preflight` first and perform a complete plan-only and
motion-only rehearsal using the same registration. The executor does not localize
the object; the following coordinates are placeholders unless independently
measured:

```bash
ros2 run ur5e_moveit_cpp plan_execute_surface_toolpath --ros-args \
  -p csv_path:="$HOME/ur5e_ws/src/ur5e_moveit_cpp/data/toolpaths/saddle/raster_along_x.csv" \
  -p planning_group:=ur_manipulator -p tcp_link:=probe_tcp \
  -p planning_frame:=world -p line_id:=-1 \
  -p object_x:=0.40 -p object_y:=0.60 -p object_z:=0.10 \
  -p planning_time:=10.0 -p planning_attempts:=8 -p approach_retries:=5 \
  -p velocity_scale:=0.05 -p acceleration_scale:=0.05 \
  -p eef_step:=0.002 -p minimum_fraction:=0.999 \
  -p simulation:=false -p execute:=false
```

For physical printing, `execute`, `hardware_confirmed`, `extrude` and
`extrusion_confirmed` must all be explicit. The configured line and total budgets
must exceed the estimate at the current pendant speed scaling or execution is
rejected before motion. Do not raise them without checking available syringe stroke
and expected deposited volume.

## Validation scope

Development checks: 23 Python tests (serial faults, geometry, budgets, trajectory
validation and executor monitor branches), nine firmware scenarios using a C++
Arduino shim, and Python syntax compilation. The monitor tests load the production
methods without ROS imports; they do not exercise ROS transport/executors.
No ROS Jazzy/MoveIt installation, AVR cross-compiler, physical UR5e or syringe was
available here. `colcon`/live actions, Xacro expansion, AVR build/upload, pressure,
mount/TCP accuracy and end-to-end stopping must be verified in the lab.

Run the offline tests from the repository root:

```bash
python3 -m unittest discover -s tests -v
python3 -m compileall -q src/surface_printing src/syringe_controller src/ur5e_toolheads
g++ -std=c++17 -I tests/firmware_stub tests/test_firmware.cpp -o /tmp/test_syringe_firmware
/tmp/test_syringe_firmware
```
