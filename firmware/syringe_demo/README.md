# Mega demo firmware

Open `syringe_demo.ino` in Arduino IDE, choose **Arduino Mega or Mega 2560**,
select its USB port and upload with the driver power off. Close Serial Monitor
before starting ROS. At 115200 baud it must print `READY SYRINGE_DEMO_V1`.
The ROS node intentionally rejects an older firmware handshake.

Pins: PUL+ 28, DIR+ 27; PUL-/DIR- common Arduino ground as in the working setup.
ENA control defaults OFF because the existing setup worked with ENA disconnected.
Do not connect ENA until its driver polarity is established. `USE_ENABLE` and
`ENABLE_ACTIVE` are compile-time settings, not emergency-stop guarantees.
The Arduino drives logic inputs, not the motor coils or motor supply.

ASCII commands, newline terminated:

- `MOVE -20 10`: 20 pulses in negative direction at 10 pulses/s, then `DONE 20`.
- `START -1 9`: continuous negative flow, bounded by all limits below.
- `PING`: renew firmware watchdog, receive `PONG`.
- `STOP`: immediate pulse stop; `ACK STOP`, `STOPPED <count>`.

Every move requires host PINGs at least every 1.5 seconds. The ROS backend sends
them at 0.2 seconds while healthy. Every continuous ROS flow also requires
`/syringe/flow_keepalive`; it stops after a 2-second lease expires. Both
`set_flow` and the low-level `set_extrusion` service are leased now.

Limits per command: 3000 pulses, 120 seconds in firmware; 110-second host cap;
finite ROS moves <=100 seconds. Invalid input, buffer overflow, missing heartbeat,
interlock trip or continuous pulse budget exhaustion latches a fault. STOP always
works but does not clear a fault. Inspect the cause, reset the Mega and relaunch
ROS. No motion resumes automatically.

Optional NC switches: set `USE_INTERLOCKS=true` only after wiring and testing
pins 30 (stop), 32 (extrusion end), 34 (retraction end), healthy contacts to GND.
An open wire trips the associated input. The default has no switches installed.
This is a software interlock, not a certified safety circuit. Provide a reachable
physical way to cut motor-driver power. A UR robot emergency stop does NOT
necessarily stop an independently powered syringe motor.

No homing/absolute plunger sensing is supplied. Pulse budgets reset per command;
repeated moves can still hit the mechanical stop. Check remaining travel before
every command. Pulse counts estimate displacement, not actual material delivered;
there is no jam/pressure/flow sensor. Test direction with an unloaded 20-pulse move
before fitting or pressing the syringe. The calibrated ROS extrusion direction
must match the firmware `EXTRUSION_DIRECTION` if directional limits are enabled.

Verification: Python serial fault-injection tests and host C++ firmware logic
checks are in `tests/`. A host shim does not validate AVR compilation, pulse timing,
electrical polarity, motor torque, hardware watchdog bootloader behavior or wiring.
