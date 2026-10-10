# Mega syringe firmware

Upload `syringe_demo.ino` using Arduino IDE, **Arduino Mega or Mega 2560**, the
correct USB port and 115200 baud. Pulling the repository does not flash the board.
Close Serial Monitor before starting ROS. The boot handshake is
`READY SYRINGE_DEMO_V1`.

PUL+ is Mega pin 28, DIR+ pin 27 and ENA+ pin 25; negative signal inputs connect
to the common signal ground in the working wiring. Enable control is ON
(`USE_ENABLE=true`), with provisional `ENABLE_ACTIVE=LOW`. Verify the driver
polarity. Normal completion disables after 300 ms; STOP and faults disable
immediately. Disabling removes holding torque.

See [the complete firmware guide](../../docs/SYRINGE_FIRMWARE.md) for every
serial command, ramps, heartbeat testing, interlocks, pulse/time limits and
fault recovery. See [the ROS command guide](../../docs/COMMANDS.md#syringe-control)
for pump services and actions. There is no encoder, homing, force or flow sensor;
pulses estimate displacement. Firmware reset and ROS restart do not establish
remaining mechanical travel.
