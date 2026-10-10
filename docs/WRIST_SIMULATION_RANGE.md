# Wrist 3 configured range

`joint_limits.yaml` currently permits -pi to +pi radians for `wrist_3_joint`.
`wrist_guard.load_bounds()` uses the same finite range for mock and physical
execution, including interpolated TCP-speed checks. Angles are not wrapped.
Earlier documentation described a hardware-only 10-degree gate; that gate is
not present in the current implementation.

This is a provisional range, not measured hardware clearance. Verify the entire
pump/needle/tubing sweep and the SRDF allowed-collision pairs before physical use.
Narrow the source bounds if the assembly needs them, rebuild
`ur5e_probe_moveit_config` and restart the stack. Confirm live MoveIt limits:

```bash
ros2 param get /move_group robot_description_planning.joint_limits.wrist_3_joint.min_position
ros2 param get /move_group robot_description_planning.joint_limits.wrist_3_joint.max_position
```

Re-add workpiece/fixture collisions and create fresh plans after model changes.
A successful sampled collision check does not establish continuous clearance.
