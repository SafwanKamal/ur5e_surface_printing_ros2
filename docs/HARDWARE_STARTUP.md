# First physical UR5e motion

This procedure stops at a 5 mm, 1 mm/s motion-only rehearsal. The saddle executor
is deliberately simulation-only until the physical saddle pose is measured and
its TCP speed is independently bounded. Do not use `confirm_mock_hardware` with a
real controller.

## 1. Pull and build

The repository root is `~/ur5e_ws` (not `~/ur5e_ws/src`). Preserve any local
calibration edits before switching branches.

```bash
cd ~/ur5e_ws
git fetch origin
git switch demo/coarse-collision
git pull --ff-only origin demo/coarse-collision

source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select \
  syringe_interfaces syringe_controller ur5e_toolheads \
  ur5e_probe_moveit_config surface_printing ur5e_moveit_cpp
source install/setup.bash
```

## 2. Extract this robot's calibration once

Connect only the ROS computer and robot, substitute the robot IP, and save the
calibration outside the repository:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:=192.168.1.102 \
  target_filename:="$HOME/ur5e_calibration.yaml"
test -s "$HOME/ur5e_calibration.yaml"
```

Do not substitute the generic default kinematics file. The real launch now makes
`kinematics_params_file` mandatory.

## 3. Physical checks before enabling the tool

With robot and pump power safely controlled:

1. Confirm the printed holder is rigid on the flange and cannot rotate or slide.
2. Confirm the displayed STL orientation matches the physical holder and syringe.
3. Measure the syringe-end TCP (needle length is currently zero) and verify it in
   RViz. Configure the pendant payload, centre of gravity, and TCP separately.
4. Check the conservative six-primitive collision envelope covers the complete
   moving apparatus, tubing, and plunger throughout their travel.
5. Place the nozzle in a clear, reachable pose with generous table/robot clearance.
6. Make the robot E-stop and independent syringe-driver power cut accessible.

Only after those checks, edit
`src/ur5e_toolheads/config/printing_tool.yaml` so these three keys are true:

```yaml
enabled: true
calibrated: true
coarse_collision: true
```

Rebuild and restart after any tool configuration change:

```bash
cd ~/ur5e_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  ur5e_toolheads ur5e_probe_moveit_config surface_printing
source install/setup.bash
```

## 4. Start the real stack

Use three terminals, each with the same environment. Do not run the mock launch at
the same time. Start the pendant External Control program and keep the speed slider
low and fixed.

Terminal A — robot driver, real robot description, MoveIt, and RViz:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur5e_ws/install/setup.bash
export ROS_DOMAIN_ID=77

ros2 launch ur5e_probe_moveit_config real_robot.launch.py \
  robot_ip:=192.168.1.102 \
  reverse_ip:=192.168.1.100 \
  kinematics_params_file:="$HOME/ur5e_calibration.yaml"
```

Terminal B — syringe safety service (required even with extrusion disabled):

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur5e_ws/install/setup.bash
export ROS_DOMAIN_ID=77
ros2 launch syringe_controller syringe.launch.py
```

The firmware must report `READY SYRINGE_DEMO_V1`. Keep the syringe empty or
mechanically disengaged for the first robot rehearsal.

Terminal C — read-only preflight:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur5e_ws/install/setup.bash
export ROS_DOMAIN_ID=77
ros2 run surface_printing hardware_preflight
```

Proceed only when it prints `PREFLIGHT PASSED`. It verifies the real UR hardware
plugin, active scaled controller, calibrated collision model, fresh joint and
speed-scaling feedback, Wrist 3 limits, and a collision-free current state. It
commands no motion.

## 5. Plan, inspect, and rehearse 5 mm without extrusion

Use the pendant to place the TCP at the verified clear start. This path captures
that current pose and moves 5 mm in world -X.

```bash
ros2 run surface_printing make_demo_path --ros-args \
  -p shape:=line -p size_mm:=5.0 -p direction:=-1.0 \
  -p frame_id:=world -p link_name:=probe_tcp

ros2 run surface_printing plan_surface_path --ros-args \
  -p csv_path:=/tmp/demo_path.csv \
  -p frame_id:=world -p link_name:=probe_tcp \
  -p cartesian_speed_mm_s:=1.0 \
  -p velocity_scale:=0.01 -p acceleration_scale:=0.01

ros2 run surface_printing execute_surface_print --ros-args \
  -p simulation:=false -p execute:=false -p extrude:=false
```

Inspect the complete displayed motion in RViz. Then, while watching the physical
robot with a hand ready at stop:

```bash
ros2 run surface_printing execute_surface_print --ros-args \
  -p simulation:=false -p execute:=true -p extrude:=false \
  -p hardware_confirmed:=true
```

The plan expires after five minutes and must start from the current joint state.
If anything moves unexpectedly, use the robot stop and cut syringe-driver power.
Do not enable extrusion or run the saddle on hardware in this first session.

## 6. Stop commands

```bash
ros2 service call /surface_printing/stop std_srvs/srv/Trigger '{}'
ros2 service call /syringe/stop std_srvs/srv/Trigger '{}'
```

These are software fallbacks, not substitutes for physical stops.
