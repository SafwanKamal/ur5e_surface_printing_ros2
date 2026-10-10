# Desktop printing console

`surface_printing_gui` is the first desktop integration of the current tools. It
runs on the Ubuntu ROS desktop using Python Tk/ttk and reuses existing executables
as subprocesses. It requires no web server, browser, or new hardware interface.
Commands are argv lists without a shell; their output appears in one console.

## Install and launch

```bash
cd ~/ur5e_ws
git fetch origin
git switch main
git pull --ff-only origin main
source /opt/ros/jazzy/setup.bash
sudo apt install python3-tk
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to surface_printing_gui
source install/setup.bash
ros2 run surface_printing_gui printing_gui \
  --workspace "$HOME/ur5e_ws" \
  --pipeline "$HOME/Surface_printing_path_generation_pipeline"
```

Clone/update the companion geometry repository and run `uv sync --locked` there
before using Geometry. `uv` must be on PATH. The selected ROS workspace must be
sourced in the parent terminal; selecting a folder in the GUI does not source it.
The GUI checks the inherited `AMENT_PREFIX_PATH` before running ROS commands.

For source-only UI development without installing the ROS package:

```bash
PYTHONPATH=src/surface_printing_gui python3 -m surface_printing_gui.app \
  --workspace "$PWD" --pipeline "$HOME/Surface_printing_path_generation_pipeline"
```

This starts the GUI but does not supply missing ROS dependencies. A graphical
session is required. If `ros2` is absent, only installed offline geometry tools
can run. Ubuntu `timeout` (coreutils) bounds stop-service clients to 8 s.

## Workflow

1. **Session:** choose mock or physical mode. Mock is the default, domain 78;
   physical mode defaults to domain 77. Confirm physical drivers are stopped and
   pump power disconnected before launching the mock robot/RViz stack. The real
   stack requires an existing robot-specific kinematics YAML. Launch the syringe
   controller separately in physical mode and run physical preflight.
2. **Geometry:** select the input mesh and a separate output file. Inspect,
   prepare, keep the largest component or normalize. Select the output as the
   next input and choose a new output name. Set the projection plane/patch,
   generate the object-mm raster, and validate it. Scaling operates on the mesh
   and CSV together. For a test saddle choose an `.stl` output filename.
3. **Path:** either capture line/circle/square/tube at the live nozzle pose, or
   convert the object-mm raster with a measured normalized-object-to-world pose.
   Conversion applies that transform once to positions and quaternions. The
   default pose values are syntax examples, not an estimated registration.
4. **Plan & Print:** choose one line, TCP speed and joint scaling. Plan to the
   saved YAML and inspect the complete trajectory in RViz. Run the check-only
   step. A successful plan **and** check for the exact same session, file contents
   and parameters unlock execution. Review/acknowledge the checklist, then start
   a mock rehearsal or explicitly selected physical motion/extrusion.
5. **Syringe:** send a small finite dispense/retract action after verifying stroke,
   direction and a clear nozzle. Pump launch/motion is disabled in mock mode.
   Negative volume retracts. No continuous-flow/heartbeat-disable UI is exposed.
6. **Diagnostics:** inspect nodes/controllers/joints/topics or start the optional
   RealSense/PCL pipeline. Centroid/bounds do not perform object registration.

The Python route plans one world-meter Cartesian line from its current start.
It does not insert a C++ OMPL approach or automatic retreat. Completed execution
consumes the UI plan gate; return safely to the start, then plan/check again.
Changing files externally also changes their fingerprints and relocks execution.
The underlying executor independently checks plan age, model, current start,
scene, speed, mode/controller, feedback and time/volume bounds.

## Process and stop handling

Robot, syringe and perception launches stay listed as owned background processes.
One foreground workflow can run at a time. The GUI refuses launches/commands in a
different session while it owns background processes from the old session. It
inherits the parent's environment and overrides only the chosen ROS domain.

**REQUEST ROBOT + PUMP STOP** sends both stop-service requests to the original
active motion session, then sends SIGINT to the owned motion client so the
existing cleanup runs. Requests are asynchronous; stdout contains responses.
Stop-service clients time out rather than waiting forever for an absent service.
Pump-only STOP does not itself stop the robot. The surface stop service exists
only while the Python executor runs.

A UI message/service reply does not prove physical cessation or pressure relief.
Keep the robot stop and independent pump power cut accessible. If a client hangs,
use the physical stops, inspect and interrupt owned processes deliberately. The
window stays open while it owns motion or background processes. It never silently
kills the robot driver when closing. Do not concurrently operate these same
hardware clients from other terminals.

## Current coverage and next integration

| Capability | GUI release 0.1 |
|---|---|
| Mock/real full-stack launch, calibration path, preflight | Included |
| Finite syringe dispense/retract and STOP | Included |
| Mesh inspect/prepare/largest-component/normalize | Included |
| Raster generation/validation, aligned scaling, test saddle | Included |
| Live line/circle/square/tube capture | Included |
| Explicit measured CSV registration | Included |
| Single-line saved-plan/check/replay | Included |
| Live process logs and owned-process interruption | Included |
| Nodes/controllers/joints/topics, RealSense/PCL launches | Included |
| 3D mesh/robot/trajectory visualization | External RViz; no embedded viewer |
| C++ multi-line saddle orchestration | CLI remains available; next integration |
| Poisson reconstruction and advanced component/normal tuning | CLI remains available |
| Tool calibration editing and firmware upload/watchdog testing | Existing guides/tools remain available |
| Saved job profiles, execution reports, registration automation | Future work |

This release integrates the conservative Python operator route first. It does
not execute the legacy C++ hard-coded motion demos, bypass collision checks,
change calibration flags, flash firmware, or guarantee material performance.

## Verification scope

`tests/test_gui_workflow.py` exercises mock/physical gates, rejected budgets,
literal argv/path handling, plan invalidation, inherited ROS-domain selection,
actual async subprocess output and SIGINT cancellation without ROS. The optional `tests/test_gui_desktop.py` smoke test uses a desktop/virtual
display and fake commands, including simulated execution and STOP clients. It
skips when DISPLAY is absent; on a normal Linux test host run
`xvfb-run -a python3 -m unittest discover -s tests -p test_gui_desktop.py -v`. Neither substitutes
for a `colcon` build and a full mock ROS/RViz rehearsal on the lab PC, followed by
bounded hardware and stop validation.
