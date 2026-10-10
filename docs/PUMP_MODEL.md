# Syringe tool model and calibration

`src/ur5e_toolheads/config/printing_tool.yaml` selects the full pump or legacy
probe in every existing robot/MoveIt Xacro. The committed configuration enables the pump, marks it calibrated, and uses
coarse collisions with a 200 mm CAD-axis extension. These saved lab values must
be verified against the current physical assembly. The demo executor requires the pump
to be enabled/calibrated for physical execution. `probe_tcp` remains the planning
chain tip for SRDF compatibility; `nozzle_tcp` is an identity alias.

The supplied STL contains 40,346 triangles. Raw bounds are approximately:
X [-45.0002,125.2498], Y [1034.4001,1124.4001], Z [72.4000,325.8834].
Assuming the CAD export is millimeters, its extent is 170.25 x 90 x 253.48 mm.
STL itself does not encode units or a mounting/TCP frame. The mesh is not
watertight; it is used as a triangle collision surface, not as a measured solid.

The current mesh translation is [0,-1.0794,-0.227868] meters, with
`mount_rpy=[pi/2,0,pi/2]`. These are saved configuration values, not a fresh
measurement of a newly assembled tool.
They must be checked against the actual flange and mounting bracket. No nozzle
position is inferred from a bounding-box extreme. The TCP is calculated from the CAD outlet plus the configured needle extension;
physical accuracy still depends on the measured mounting and actual nozzle. The STL only represents one plunger
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

The SRDF allows several fixed mounting/internal-envelope contacts, including
sleeve-to-wrist_2/wrist_3, body-to-wrist_3, and axial-nozzle-to-sleeve/wrist_3.
Inspect `config/ur5e_with_probe_mock.srdf` for the complete pair list. These pairs
are ignored at all angles; confirm they represent actual fixed contacts and do
not hide moving-tool collisions. Do not add exclusions to force a failing plan.

The same installed configuration must be used by MoveIt and robot_state_publisher.
The executor checks live MoveIt TCP/mount/mesh transforms against that file and
requires a new plan when the live robot model hash changes. It cannot certify
that those values match the real hardware.

## CAD outlet TCP and needle extension

The orthographic STL inspection identifies the narrow cylindrical outlet above
 the barrel, along CAD +Z. The terminal planar ring has 48 unique vertices, a
4.10 mm outer diameter, and center [96.74983, 1081.40020, 325.88336] mm.
This is the syringe outlet without a needle, not a measured physical calibration.
With the current mesh placement its tool-frame position is
[0.09674983, 0.00200020, 0.09801536] m. TCP +Z follows the outlet axis.
That coordinate is the outlet with zero extension. The committed 200 mm extension
places `probe_tcp` at [0.09674983,0.00200020,0.29801536] m in the tool frame.
Keep calibrated=false after changes until the full assembly is verified.

To configure the needle, measure the actual outlet-to-tip extension when fitted
(not the complete needle length including its overlapping hub). From the source
repository root, preview a 25 mm extension:

```bash
python3 src/ur5e_toolheads/ur5e_toolheads/set_needle_tcp.py \
  --config src/ur5e_toolheads/config/printing_tool.yaml \
  --needle-length-mm 25
```

Add `--write` to apply. Use `--needle-length-mm 0 --write` for no needle.
After building, `ros2 run ur5e_toolheads set_needle_tcp` accepts the same arguments.
The script computes mesh_translation + mesh_rotation * (scaled_CAD_outlet +
[0, 0, needle_length_m]); it handles rotated mesh axes and never accumulates offsets.
It updates both tcp_xyz and tcp_rpy and resets calibrated=false. It does not change
mounting, enable the pump, move the robot, or publish a competing TF.

Stop execution before editing. Verify the model and physical tool, set calibration
only after verification, rebuild ur5e_toolheads, and restart robot/MoveIt/RViz.
Regenerate the path from the new tip and make a fresh plan. Updating a standalone
TF during execution would leave MoveIt's kinematic model inconsistent.
In coarse collision mode, a positive CAD needle extension adds a cylindrical
`pump_needle_envelope`. Default radius is 1 mm, with 1 mm base and 2 mm tip
clearances; verify these against the fitted needle/hub. The alternate axial nozzle
has its own measured-radius collision cylinder. Full-mesh collision mode does not
automatically add the CAD-extension cylinder. Allow clearance for the complete
physical tool, including omitted tip gaps, hub, cables and moving plunger. Update the pendant TCP separately.

## Alternate axial nozzle

`set_axial_nozzle_tcp` configures total `tool0`-to-tip length along tool0 +Z,
not an extension from the CAD syringe outlet. It requires a measured outer radius
and retains the existing pump geometry. Preview before `--write`, then rebuild,
restart and verify. See [COMMANDS.md](COMMANDS.md#tool-configuration) and the
[tube guide](../TUBE_DEMO.md). Do not use both configuration conventions blindly;
inspect the resulting YAML and TF for the intended physical tool.
