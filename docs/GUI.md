# Desktop printing console

`surface_printing_gui` is the first desktop integration of the current tools. It
runs on the Ubuntu ROS desktop using Python Tk/ttk and reuses existing executables
as subprocesses. It requires no web server, browser, or new hardware interface.
Commands are argv lists without a shell; their output appears in one console.

## Install and launch

For a clean checkout, run the block below. The subshell stops on the first
failure, so a failed branch switch cannot silently build the old branch.

```bash
(
  set -e
  cd ~/ur5e_ws
  git fetch origin
  git switch main
  git merge --ff-only origin/main
  source /opt/ros/jazzy/setup.bash
  sudo apt install python3-tk
  rosdep install --from-paths src --ignore-src -y --skip-keys warehouse_ros_mongo
  colcon build --symlink-install --packages-up-to surface_printing_gui
  source install/setup.bash
  ros2 run surface_printing_gui printing_gui \
    --workspace "$HOME/ur5e_ws" \
    --pipeline "$HOME/Surface_printing_path_generation_pipeline"
)
```

`warehouse_ros_mongo` is the MongoDB backend declared by the generated MoveIt
configuration. Skip it for the GUI/robot workflow when rosdep cannot resolve it.
This does not install the backend needed by `warehouse_db.launch.py`; install that
separately if you use the database warehouse. Other unresolved dependencies stop
this installation block.

### Recover when a local tool calibration blocks switching to main

If Git names `src/ur5e_toolheads/config/printing_tool.yaml` as the modified file,
use this update block first. It copies your exact calibration outside the repo,
stashes only that file, switches to main, and restores your calibration. It keeps
the stash as a second backup and retains commits on your previous branch.

```bash
(
  set -e
  cd ~/ur5e_ws
  ur5e_backup_dir=$(mktemp -d "$HOME/ur5e-config-backup.XXXXXX")
  cp -- src/ur5e_toolheads/config/printing_tool.yaml "$ur5e_backup_dir/printing_tool.yaml"
  printf 'Calibration backup: %s\n' "$ur5e_backup_dir/printing_tool.yaml"
  git stash push -m "Preserve local tool calibration before GUI update" -- \
    src/ur5e_toolheads/config/printing_tool.yaml
  git fetch origin
  git switch main
  git merge --ff-only origin/main
  cp -- "$ur5e_backup_dir/printing_tool.yaml" src/ur5e_toolheads/config/printing_tool.yaml
  test -f src/surface_printing_gui/package.xml
)
```

If this stops at a Git step, the backup and stash remain available. Do not reset
or discard calibration to force the update. Inspect `git status --short`,
`git branch --show-current`, and `git log --oneline --left-right main...origin/main`
before resolving a divergent local main or other modified files. After a
successful recovery, run the install block above; restoring calibration while
already on main will not prevent `git switch main`.

Clone/update the companion geometry repository and run `uv sync --locked` there
before using Geometry. `uv` must be on PATH. The selected ROS workspace must be
sourced in the parent terminal; selecting a folder in the GUI does not source it.
The GUI checks the inherited `AMENT_PREFIX_PATH` before running ROS commands.

### Recover a stale toolheads build after the configuration archive move

An incremental symlink build from an older checkout can fail with
`can't copy .../build/ur5e_toolheads/config/printing_tool.pre_calibrated_backup.yaml`.
That archived snapshot now lives under `docs/archive/tool-config`; an old build
directory can still contain a dangling link matched by the configuration glob.
Move aside only this package's generated build/install folders and rebuild:

```bash
(
  set -e
  cd ~/ur5e_ws
  ur5e_build_backup=$(mktemp -d "$HOME/ur5e-build-backup.XXXXXX")
  for ur5e_generated_dir in build/ur5e_toolheads install/ur5e_toolheads; do
    if [ -e "$ur5e_generated_dir" ] || [ -L "$ur5e_generated_dir" ]; then
      mv -- "$ur5e_generated_dir" "$ur5e_build_backup/$(dirname "$ur5e_generated_dir")-ur5e_toolheads"
    fi
  done
  source /opt/ros/jazzy/setup.bash
  colcon build --symlink-install --packages-up-to surface_printing_gui
  source install/setup.bash
  ros2 run surface_printing_gui printing_gui \
    --workspace "$HOME/ur5e_ws" \
    --pipeline "$HOME/Surface_printing_path_generation_pipeline"
)
```

This retains the old generated folders as backups. It does not move or edit
`src/ur5e_toolheads/config/printing_tool.yaml` or clear unrelated packages.

For source-only UI development without installing the ROS package:

```bash
PYTHONPATH=src/surface_printing_gui python3 -m surface_printing_gui.app \
  --workspace "$PWD" --pipeline "$HOME/Surface_printing_path_generation_pipeline"
```

This starts the GUI but does not supply missing ROS dependencies. A graphical
session is required. If `ros2` is absent, only installed offline geometry tools
can run. Ubuntu `timeout` (coreutils) bounds stop-service clients to 8 s.

## Workflow

### Dedicated mock motion verification

Select **mock**, domain **78**, then click **Open mock verification** in the main
toolbar. This opens a separate window pinned to that workspace/domain. Its
controls share path/plan settings with the main tabs and reuse their plan/check
gate. It never enables pump launch or extrusion.

1. Confirm physical drivers are stopped and pump power is disconnected.
2. Click **Launch mock robot + RViz** unless that stack already runs on domain 78.
3. Click **Connect live mock state**. Wait for **LIVE** joint/TF feedback. The
   read-only `mock_monitor` executable subscribes to `/joint_states` and TF;
   it refuses domains other than 78. No joint or motion commands originate from
   the telemetry node.
4. Use RViz to position the mock tip at the desired start. A tube requires the
   nozzle downward within 15 degrees. Choose shape/size and tube dimensions,
   then **1. Capture at current mock tip**. The captured tube tip is on the
   circumference; its center is diameter/2 toward world -X.
5. Click **2. Plan + RViz preview** and review the complete arm/tool trajectory in
   RViz. Click **3. Check saved plan** and acknowledge the review checkbox.
6. Click **4. Rehearse on mock hardware**. Blue is the requested world-meter CSV
   path; orange is the live TCP trace collected during that run. Green is the
   current fresh TCP; gray indicates stale state. Top X/Y and side X/Z views keep
   equal scale for the plotted axes. Joint angles come from live joint states.

The progress bar estimates elapsed/planned time and stays below 100% while the
executor runs. A completed result requires a successful executor exit plus fresh
joint/TF samples during the run; it does not assert geometric accuracy, complete
sample coverage or physical feasibility. Missing/stale feedback produces an
unverified result, not a simulated success. Review the trace and RViz yourself.
Changed files/settings require another plan/check. The monitor samples at 5 Hz;
the trace is a sampled projection, not a full robot collision visualizer.

**REQUEST MOCK ROBOT STOP** stays on the window's original mock domain, even if
the main Session selection changes. A changed session disables its motion
controls. Closing the mock window interrupts its read-only monitor; it leaves
the robot stack/any owned executor visible in the main Session list. Use the
stop controls and deliberately shut down the mock stack before launching the
physical stack. Capture, plan and check again in the physical session; mock
rehearsal does not establish physical calibration or unlock physical execution.

### Main workflow

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
