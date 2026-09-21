# Wrist 3 simulation range

The mock demo now uses -pi to +pi radians (-180 to +180 degrees) for wrist_3_joint instead of +/-5 degrees. This is a bounded simulation experiment, not validated hardware clearance. The Python planner, trajectory checks, interpolated TCP-speed checks and simulation executor use the same installed joint_limits.yaml. Angles are not wrapped.

No collision geometry or allowed-collision pairs are changed. Allowed pairs remain ignored at all angles; a successful plan therefore does not establish physical clearance. The Python hardware executor rejects a configured range wider than 10 degrees. Existing calibration and mock-model guards remain in effect.

Rebuild ur5e_probe_moveit_config and surface_printing, restart the demo, re-add the saddle collision object and create fresh plans. Check the live MoveIt robot_description_planning.joint_limits.wrist_3_joint.min_position and max_position parameters; expected values are -3.141592653589793 and +3.141592653589793. Test the saddle approach with execute:=false first.
