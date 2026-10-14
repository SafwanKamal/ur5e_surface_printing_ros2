# Independent motion-controller startup

The lab startup log showed the active grouped spawner loading broadcasters and
ur_configuration_controller, then failing at gravity_update_controller before
it reached scaled_joint_trajectory_controller. The inactive group separately
failed at twist_controller. No spawner remained running afterward.

real_robot.launch.py now includes ur_control_independent.launch.py. That wrapper
uses the installed ur_robot_driver launch with a restricted AST transformation:
the selected initial_joint_controller is removed from the grouped active/inactive
lists and spawned separately using the vendor controller_spawner factory.
Controller YAML parameters, activation flag, manager timeout, hardware model
and all other driver launch actions remain vendor-provided. The controller
stopper remains active and unmodified. It can stop controllers when External
Control stops and reactivate its remembered controllers when the program resumes.
The wrapper does not continually reactivate controllers or resume trajectories.

Optional-controller failures remain visible and need installation diagnosis;
this change only prevents those failures from blocking the printing controller.
Incompatible vendor launch structure fails explicitly rather than silently
running the old grouped startup.

## Apply

Stop current execution and the existing real robot launch first. Update/build:

```bash
cd ~/ur5e_ws && git pull --ff-only origin main && \
source /opt/ros/jazzy/setup.bash && \
colcon build --symlink-install --packages-select ur5e_probe_moveit_config && \
source install/setup.bash
```

Restart the usual driver:

```bash
ROS_DOMAIN_ID=77 ros2 launch ur5e_probe_moveit_config real_robot.launch.py \
  robot_ip:=192.168.1.102 reverse_ip:=192.168.1.100 \
  kinematics_params_file:="$HOME/ur5e_calibration.yaml"
```

Press Play on the pendant External Control program, then check:

```bash
ROS_DOMAIN_ID=77 ros2 control list_controllers
ROS_DOMAIN_ID=77 ros2 run surface_printing hardware_preflight
```

The startup should now independently log the scaled controller load/configure/
activation. If it subsequently deactivates despite External Control being active,
inspect the driver log for the state transition; no launch change should blindly
reactivate it during an ongoing print.

## Separate installation errors

The same log showed dashboard_client and urscript_interface exiting with code
127. This often indicates dynamic-loader problems, but the log excerpt did not
include the cause. Run:

```bash
ldd /opt/ros/jazzy/lib/ur_robot_driver/dashboard_client
ldd /opt/ros/jazzy/lib/ur_robot_driver/urscript_interface
```

Look for missing libraries and inspect each executable's stderr. Missing optional
controller plugins can indicate that driver launch/config files and installed
controller plugins are out of sync. Repair matching ROS packages or library
resolution after identifying the missing dependency; this wrapper does not fix
or conceal those installation errors.

## Verification

Offline tests verify independent controller loading, preservation of
activate_joint_controller=false, retention of optional groups, no duplicate
selected controller, and explicit rejection of an unsupported vendor layout.
The transformation also compiles against the upstream Jazzy launch. Actual
startup and controller-manager behavior must be verified on the lab Ubuntu PC.
