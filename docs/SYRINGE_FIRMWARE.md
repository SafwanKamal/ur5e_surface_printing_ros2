# Ramped syringe firmware

Upload `firmware/syringe_demo/syringe_demo.ino` to an Arduino Mega 2560.
Updating or pulling the repository does not flash the board. Close Serial
Monitor before launching ROS; use 115200 baud and Newline for manual commands.

PUL+ is pin 28, DIR+ is pin 27, and ENA+ is pin 25. ENA- must connect to the
signal ground for the configured wiring. `ENABLE_ACTIVE=LOW` is provisional:
confirm your driver's polarity; set HIGH if it enables on HIGH. The driver model
has not been identified. Firmware STATUS reports the commanded state only.

## Commands

| Command | Behavior |
| --- | --- |
| `MOVE -2000 1000` | Exactly 2000 negative driver pulses, ramping toward 1000 Hz and back down |
| `START 1 1000` | Ramp positive toward 1000 Hz; still bounded by pulse and run-time budgets |
| `STOP` | Immediate pulse shutdown and driver disable; unchanged ROS stop behavior |
| `ESTOP` | Same immediate shutdown, with `ACK ESTOP` |
| `RAMP_STOP` | Decelerate to starting speed, then stop; extra pulses occur during braking |
| `STATUS` | Motion, unsigned pulse count, current/target rate, commanded driver state, fault latch |
| `PING` | Refresh serial heartbeat; replies `PONG` |
| `WATCHDOG OFF` | Disable only the serial heartbeat watchdog for manual testing |
| `WATCHDOG ON` | Re-enable heartbeat and refresh its timestamp |
| `WATCHDOG STATUS` | Report heartbeat state and timeout in ms |
| `WATCHDOG TIMEOUT 5000` | Set heartbeat timeout; accepted range 100..60000 ms |

Commands are case-insensitive. `ZERO` is not supported: this protocol counts
pulses per command rather than maintaining a global position. `DONE`, `PROGRESS`,
and `STOPPED` retain unsigned per-command counts and the boot line remains
`READY SYRINGE_DEMO_V1`. No encoder verifies actual motion.

## Profile and limits

The starting speed is min(requested rate, 100 Hz), and acceleration/deceleration
is 1000 driver pulses/s². The speed of pulse index i in an N-pulse finite move is
`min(target_rate, sqrt(start_rate² + 2*acceleration*min(i, N-1-i)))`.
Long moves have a plateau; short moves have a triangular profile. These constants
are initial tuning values, not validated motor limits. At rates <=100 Hz there
is no ramp above the requested speed. No catch-up bursts are generated.

Enable settles for 1 ms before the first pulse. Normal completion/`RAMP_STOP`
retains holding torque for 300 ms, then disables. STOP, ESTOP and faults disable
immediately. Check whether syringe pressure can back-drive the unpowered plunger.

The 3000-pulse budget, 5000 Hz command cap, 120-second firmware run cap, optional
interlocks, latched faults and 2-second AVR reset watchdog remain. Continuous
START also stops with `FAULT PULSE_BUDGET` at 3000 pulses. WATCHDOG OFF does not
remove these protections or clear a latched fault. Reset restores heartbeat ON
with the original 1500 ms timeout. Invalid commands still latch a fault.

Manual unloaded test:

```text
WATCHDOG OFF
MOVE -1000 500
```

Wait for `DONE 1000`, then test the reverse direction. Use STOP to abort. After
manual testing, send `WATCHDOG ON`; with it enabled, send PING about every 0.2–0.5
seconds during longer motion. Continuous motion with heartbeat disabled continues
until a stop, pulse budget, run limit or another fault.

ROS startup explicitly restores WATCHDOG TIMEOUT 1500 and WATCHDOG ON; it does
not offer a watchdog-disable API. Move deadlines and the 100-second goal limit
now use the ramp duration plus the existing deadline margin. Flow-service values
remain the target plateau flow, not the average flow during acceleration.
If changing profile constants, update `motion_profile.py` to match and rerun tests.

## Installation

After checking out the integration branch on the desktop, upload the sketch in
Arduino IDE (Mega 2560 and the correct port), then:

```bash
cd ~/ur5e_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select syringe_controller --symlink-install
source install/setup.bash
```

Pulse generation is software-timed. Offline host tests validate counts, ramps,
stop behavior and protocol, not driver polarity, AVR execution timing, motor
following or physical extrusion. Validate at low speed unloaded before pressure
loads. Existing fault reset requirements and physical interlock setup still apply.

Validation on demo/coarse-collision: 54 Python tests and the firmware host-simulation
checks passed. The sketch and complete Arduino AVR core 1.8.6 compiled and linked
for ATmega2560 with that core's indexed AVR GCC toolchain: 11140 bytes program,
1189 bytes static RAM. No physical motor/driver or ROS hardware session was tested.
