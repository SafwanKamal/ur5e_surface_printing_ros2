# Verified abort recovery

The observed failure combined an early measured TCP-speed excursion with
unconfirmed MoveIt cancellation. Later path captures changed position and
planning reported 4-5 mm start mismatches, consistent with the old trajectory
continuing to run after the executor exited.

## Changes

Abort cleanup dispatches independent requests before waiting on any one service:
MoveIt cancellation, cancellation of all goals on the physical scaled controller
FollowJointTrajectory action, motion-controller deactivation, and syringe STOP.
Simulation deactivates its mock controller rather than contacting the real
controller action service.

Cleanup checks controller state and fresh observed joint positions. It reports
ABORT STOP VERIFIED only when the motion controller is inactive and joints remain
stationary for 0.5 seconds. Otherwise it reports ROBOT STOP NOT VERIFIED and
directs use of the physical pendant stop. This is software confirmation, not a
safety-rated physical stop guarantee.

An abort deliberately leaves the motion controller inactive; successful prints
do not. Check physical robot state before reactivating after a fault. The normal
automatic startup configuration remains unchanged.

Current-position generators, the planner, and execution start checks use fresh
joint feedback directly. They do not wait for a stationary-joint observation
window. The stationary check is retained only for confirming an abort stop.
No return to an earlier starting position is commanded.

## Measured-speed estimates

The planned speed ceiling and 90% target-band coverage checks are unchanged.
Live finite-difference TCP estimates use 0.25-second windows and are report-only.
Measured excess speed and insufficient measured coverage do not trigger aborts.
They are logged for diagnosis and material-quality assessment.
Feedback validity and the unrelated robot/syringe fault checks remain active.

## Recovery

If robot motion continues, use the pendant stop. In a sourced domain-77 terminal:

```bash
ROS_DOMAIN_ID=77 ros2 control switch_controllers \
  --deactivate scaled_joint_trajectory_controller --strict
ROS_DOMAIN_ID=77 ros2 service call /syringe/stop std_srvs/srv/Trigger '{}'
```

After the robot is stopped, update/build surface_printing and source the workspace.
Then check physical robot state before reactivation:

```bash
ROS_DOMAIN_ID=77 ros2 control switch_controllers \
  --activate scaled_joint_trajectory_controller --strict
ROS_DOMAIN_ID=77 ros2 run surface_printing hardware_preflight
```

Regenerate from the stationary current position and replan; do not reuse the
previous YAML after motion or a failed plan. Run the first fresh trajectory with
extrude=false to verify live speed tracking before depositing material.

## Offline validation

Tests reproduce a MoveIt cancellation timeout, independent controller-stop
dispatch, rejection of unconfirmed stationary feedback, and no controller
deactivation when this executor has not submitted motion. They also verify that
isolated and sustained speed readings remain report-only. Existing timing, executor,
geometry and generator checks are run. Real ROS transport and physical stopping
behavior have not been tested in the development container.
