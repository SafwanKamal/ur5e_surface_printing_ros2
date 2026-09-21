# Syringe tool model and calibration

`src/ur5e_toolheads/config/printing_tool.yaml` selects the full pump or legacy
probe in every existing robot/MoveIt Xacro. Defaults retain the probe (`enabled:
false`) and mark the new pump uncalibrated. The demo executor requires the pump
to be enabled/calibrated for physical execution. `probe_tcp` remains the planning
chain tip for SRDF compatibility; `nozzle_tcp` is an identity alias.

The supplied STL contains 40,346 triangles. Raw bounds are approximately:
X [-45.0002,125.2498], Y [1034.4001,1124.4001], Z [72.4000,325.8834].
Assuming the CAD export is millimeters, its extent is 170.25 x 90 x 253.48 mm.
STL itself does not encode units or a mounting/TCP frame. The mesh is not
watertight; it is used as a triangle collision surface, not as a measured solid.

The mesh translation [0,-1.0794,-0.227868] meters and mounting rotation about Y
are inherited from the old probe as a STARTING GUESS, based on shared CAD offsets.
They must be checked against the actual flange and mounting bracket. No nozzle
position is inferred from a bounding-box extreme. The TCP defaults to zero and
is intentionally unusable until configured. The STL only represents one plunger
position; account for its entire operating stroke and cables when checking space.

## Set up

1. Determine STL units and flange mounting orientation from the CAD assembly.
2. Edit `mount_xyz/rpy` (flange to tool frame) and `mesh_xyz/rpy/scale` (CAD mesh
   placement inside that frame). Meters and radians, except the dimensionless scale.
3. Measure/calibrate the actual nozzle tip, including the fitted needle. Set
   `tcp_xyz/rpy` relative to `probe_tool_link`. If your measured TCP is relative to
   flange, transform it into the tool frame; do NOT paste flange coordinates here.
   Use T_tool_tcp = inverse(T_flange_tool) * T_flange_tcp.
4. Set `enabled: true`, rebuild `ur5e_toolheads` and restart ALL robot description,
   MoveIt and RViz launches. Visually verify flange, pump and nozzle alignment,
   including orientation, against the physical tool. Set `calibrated: true` only
   after this is verified; rebuild and restart again.
5. Confirm `/compute_fk` for `probe_tcp` matches the physical nozzle pose. Set the
   robot's actual tool payload/center of gravity and pendant TCP separately using
   measured values. The mesh supplies no mass/inertia or payload configuration.
6. Include table, fixtures and object in the MoveIt scene. Collision checking
   cannot protect against unmodeled geometry. Use a positive nozzle standoff.

The previous SRDF exclusions between the tool and wrist_1/wrist_2 are removed
because they were based on the small probe. The flange-adjacent wrist_3 exclusion
remains; inspect that interface physically. Do not disable more collision pairs
just to force a failing plan to execute.

The same installed configuration must be used by MoveIt and robot_state_publisher.
The executor checks live MoveIt TCP/mount/mesh transforms against that file and
requires a new plan when the live robot model hash changes. It cannot certify
that those values match the real hardware.
