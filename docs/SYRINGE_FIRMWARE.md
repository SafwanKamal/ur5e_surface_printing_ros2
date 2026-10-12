# Ramped syringe firmware

## Working V3 sketch (recommended)

`firmware/syringe_stepper_ramped_enable/syringe_stepper_ramped_enable.ino`
is based on the working sketch supplied by Safwan. Its pulse engine, 10 ms
enable settling, 100 Hz starting speed, 1000 pulses/s² time-based ramp,
250 ms progress reporting, and 20-second heartbeat watchdog are preserved.
The original 2,000,000-pulse limit is removed; nonzero signed 32-bit MOVE
arguments remain. MOV is a MOVE alias. No frequency-to-flow conversion is
needed for `/syringe/move_plunger`.

ROS detects `READY SYRINGE_STEPPER_RAMP_ENABLE_V3`, sends MOVE/START unchanged,
and supplies PING internally. It does not send WATCHDOG configuration commands
to V3. An already flashed working V3 sketch is supported without reflashing
(that original sketch retains its own 2,000,000 limit).

| V3 command | Meaning |
| --- | --- |
| `MOVE -2000 200` | Finite ramped move; signed PROGRESS and DONE counts |
| `START -1 200` | Continuous ramped motion; maintain PING |
| `STOP` | Ramp down, ACK STOPPING; STOPPED POS arrives after stopping |
| `ESTOP` | Immediate stop and driver disable; ACK ESTOP |
| `PING` | Heartbeat and PONG |
| `STATUS` | Mode, position, rate, target, direction and driver state |
| `ZERO` | Reset reported position while idle |

The ROS `/syringe/stop` endpoint uses ESTOP for V3 to preserve immediate-stop
semantics. RESET/READY, malformed feedback and firmware faults still latch in
ROS; STOP acknowledgement alone does not clear them. V3 has no WATCHDOG
configuration command or fixed firmware run-time cap. Host moves retain their
100-second admission limit and heartbeat-loss detection.

## Legacy DEMO_V1 sketch

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

The 5000 Hz command cap, 120-second firmware run cap, optional
interlocks, latched faults and 2-second AVR reset watchdog remain. Continuous
The fixed 3000-pulse cap is removed for MOVE and START. WATCHDOG OFF does not
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

After updating `main` on the desktop, upload the sketch in
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

Historical validation before integration: 54 Python tests and the firmware host-simulation
checks passed. The sketch and complete Arduino AVR core 1.8.6 compiled and linked
for ATmega2560 with that core's indexed AVR GCC toolchain: 11140 bytes program,
1189 bytes static RAM. No physical motor/driver or ROS hardware session was tested.
