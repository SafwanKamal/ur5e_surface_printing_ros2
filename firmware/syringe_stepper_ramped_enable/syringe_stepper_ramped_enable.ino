// Based on the working V3 sketch supplied by Safwan Kamal.
// Pulse timing, ramp, enable settling and 20-second watchdog preserved.
#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
namespace {
constexpr uint8_t DIRECTION_PIN = 27;
constexpr uint8_t PULSE_PIN = 28;
constexpr uint8_t ENABLE_PIN = 25;
// IMPORTANT: Confirm polarity with YOUR driver manual before operating.
// Assumes ENA- is connected to Arduino GND, and HIGH on ENA+
// activates the driver's disable input (common for some opto-isolated drivers).
// Reverse both levels if your driver's behavior is opposite.
constexpr uint8_t DRIVER_ENABLE_LEVEL = LOW;
constexpr uint8_t DRIVER_DISABLE_LEVEL = HIGH;
constexpr unsigned long ENABLE_SETTLE_MS = 10;
constexpr unsigned long IDLE_DISABLE_DELAY_MS = 300;
constexpr unsigned long BAUD_RATE = 115200;
constexpr unsigned long MINIMUM_RATE_HZ = 1;
constexpr unsigned long MAXIMUM_RATE_HZ = 5000;
constexpr unsigned long WATCHDOG_TIMEOUT_MS = 20000;
constexpr unsigned long PROGRESS_PERIOD_MS = 250;
constexpr size_t COMMAND_BUFFER_SIZE = 96;
// Tune for your motor, current limit, voltage, and mechanical load.
// Units: driver STEP pulses per second and pulses per second squared.
constexpr float START_RATE_HZ = 100.0f;
constexpr float ACCEL_STEPS_PER_S2 = 1000.0f;
constexpr unsigned long RAMP_UPDATE_US = 10000UL;  // Re-evaluate ramp at 100 Hz.
enum class MotionMode { IDLE, CONTINUOUS, FINITE, STOPPING };
MotionMode motion_mode = MotionMode::IDLE;
char command_buffer[COMMAND_BUFFER_SIZE];
size_t command_length = 0;
bool pulse_is_high = false;
bool driver_enabled = false;
unsigned long idle_since_ms = 0;
bool finish_after_falling_edge = false;
int direction_sign = 1;
unsigned long half_period_us = 5000;
unsigned long last_edge_us = 0;
unsigned long last_ramp_us = 0;
unsigned long last_heartbeat_ms = 0;
unsigned long last_progress_ms = 0;
unsigned long remaining_steps = 0;
unsigned long completed_move_steps = 0;
unsigned long target_rate_hz = 0;
float actual_rate_hz = 0.0f;
long commanded_move_steps = 0;
long position_steps = 0;
const __FlashStringHelper* motionModeName() {
  switch (motion_mode) {
    case MotionMode::CONTINUOUS: return F("CONTINUOUS");
    case MotionMode::FINITE: return F("FINITE");
    case MotionMode::STOPPING: return F("STOPPING");
    default: return F("IDLE");
  }
}
void setPulseLow() {
  digitalWrite(PULSE_PIN, LOW);
  pulse_is_high = false;
}
void disableDriver() {
  // STEP must be LOW before disabling the winding drive.
  setPulseLow();
  digitalWrite(ENABLE_PIN, DRIVER_DISABLE_LEVEL);
  driver_enabled = false;
}
void enableDriver() {
  if (!driver_enabled) {
    digitalWrite(ENABLE_PIN, DRIVER_ENABLE_LEVEL);
    driver_enabled = true;
    delay(ENABLE_SETTLE_MS);
  }
}
void serviceIdleDisable() {
  if (motion_mode == MotionMode::IDLE && driver_enabled &&
      millis() - idle_since_ms >= IDLE_DISABLE_DELAY_MS) {
    disableDriver();
  }
}
void stopMotion(bool report) {
  setPulseLow();
  motion_mode = MotionMode::IDLE;
  finish_after_falling_edge = false;
  remaining_steps = 0;
  target_rate_hz = 0;
  actual_rate_hz = 0.0f;
  idle_since_ms = millis();
  if (report) {
    Serial.print(F("STOPPED POS "));
    Serial.println(position_steps);
  }
}
bool parseLong(const char* token, long& value) {
  if (!token || !*token) return false;
  char* end = nullptr;
  errno = 0;
  value = strtol(token, &end, 10);
  return !errno && end != token && *end == '\0' && value >= INT32_MIN && value <= INT32_MAX;
}
bool validRate(long rate_hz) {
  return rate_hz >= (long)MINIMUM_RATE_HZ && rate_hz <= (long)MAXIMUM_RATE_HZ;
}
void updateHalfPeriod() {
  // Minimum 100 us half-period at 5 kHz, above common STEP input minima.
  unsigned long period = (unsigned long)(500000.0f / actual_rate_hz);
  half_period_us = period > 0 ? period : 1;
}
void beginMotion(MotionMode mode, int requested_direction,
                 unsigned long rate_hz, unsigned long step_count,
                 long signed_step_count) {
  setPulseLow();
  enableDriver();
  direction_sign = requested_direction >= 0 ? 1 : -1;
  digitalWrite(DIRECTION_PIN, direction_sign > 0 ? HIGH : LOW);
  delayMicroseconds(10);
  target_rate_hz = rate_hz;
  actual_rate_hz = min((float)rate_hz, START_RATE_HZ);
  updateHalfPeriod();
  remaining_steps = step_count;
  completed_move_steps = 0;
  commanded_move_steps = signed_step_count;
  finish_after_falling_edge = false;
  last_edge_us = micros();
  last_ramp_us = last_edge_us;
  last_heartbeat_ms = millis();
  last_progress_ms = last_heartbeat_ms;
  motion_mode = mode;
}
void printStatus() {
  Serial.print(F("STATE "));
  Serial.print(motionModeName());
  Serial.print(F(" POS "));
  Serial.print(position_steps);
  Serial.print(F(" RATE "));
  Serial.print(actual_rate_hz, 1);
  Serial.print(F(" TARGET "));
  Serial.print(target_rate_hz);
  Serial.print(F(" DIR "));
  Serial.print(direction_sign);
  Serial.print(F(" DRIVER "));
  Serial.println(driver_enabled ? F("ENABLED") : F("DISABLED"));
}
void handleCommand(char* command) {
  for (char* p = command; *p; ++p) *p = (char)toupper((unsigned char)*p);
  char* name = strtok(command, " \t");
  if (!name) return;
  last_heartbeat_ms = millis();
  if (strcmp(name, "PING") == 0) { Serial.println(F("PONG")); return; }
  if (strcmp(name, "STATUS") == 0) { printStatus(); return; }
  if (strcmp(name, "ESTOP") == 0) {
    stopMotion(false);
    disableDriver();
    Serial.println(F("ACK ESTOP"));
    Serial.print(F("STOPPED POS "));
    Serial.println(position_steps);
    return;
  }
  if (strcmp(name, "STOP") == 0) {
    if (motion_mode == MotionMode::IDLE) {
      Serial.println(F("ACK STOP"));
      Serial.print(F("STOPPED POS "));
      Serial.println(position_steps);
    } else {
      motion_mode = MotionMode::STOPPING;
      finish_after_falling_edge = false;
      Serial.println(F("ACK STOPPING"));
    }
    return;
  }
  if (strcmp(name, "ZERO") == 0) {
    if (motion_mode != MotionMode::IDLE) { Serial.println(F("ERROR BUSY")); return; }
    position_steps = 0;
    Serial.println(F("ACK ZERO"));
    return;
  }
  if (strcmp(name, "START") == 0) {
    long direction = 0, rate = 0;
    bool ok = parseLong(strtok(nullptr, " \t"), direction) &&
              parseLong(strtok(nullptr, " \t"), rate) &&
              strtok(nullptr, " \t") == nullptr;
    if (!ok || (direction != 1 && direction != -1)) {
      Serial.println(F("ERROR USAGE START <1|-1> <RATE_HZ>")); return;
    }
    if (!validRate(rate)) { Serial.println(F("ERROR RATE_OUT_OF_RANGE")); return; }
    if (motion_mode != MotionMode::IDLE) { Serial.println(F("ERROR BUSY")); return; }
    beginMotion(MotionMode::CONTINUOUS, (int)direction, (unsigned long)rate, 0, 0);
    Serial.println(F("ACK START"));
    return;
  }
  if (strcmp(name, "MOVE") == 0 || strcmp(name, "MOV") == 0) {
    long steps = 0, rate = 0;
    bool ok = parseLong(strtok(nullptr, " \t"), steps) &&
              parseLong(strtok(nullptr, " \t"), rate) &&
              strtok(nullptr, " \t") == nullptr;
    if (!ok || steps == 0 || steps == INT32_MIN) {
      Serial.println(F("ERROR USAGE MOVE <SIGNED_STEPS> <RATE_HZ>")); return;
    }
    if (!validRate(rate)) { Serial.println(F("ERROR RATE_OUT_OF_RANGE")); return; }
    unsigned long magnitude = steps > 0 ? (unsigned long)steps :
                              (unsigned long)(-(steps + 1)) + 1UL;
    if (motion_mode != MotionMode::IDLE) { Serial.println(F("ERROR BUSY")); return; }
    beginMotion(MotionMode::FINITE, steps > 0 ? 1 : -1,
                (unsigned long)rate, magnitude, steps);
    Serial.println(F("ACK MOVE"));
    return;
  }
  Serial.println(F("ERROR UNKNOWN_COMMAND"));
}
void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      command_buffer[command_length] = '\0';
      handleCommand(command_buffer);
      command_length = 0;
      continue;
    }
    if (command_length + 1 >= COMMAND_BUFFER_SIZE) {
      command_length = 0;
      Serial.println(F("ERROR COMMAND_TOO_LONG"));
      continue;
    }
    command_buffer[command_length++] = c;
  }
}
void serviceRamp() {
  if (motion_mode == MotionMode::IDLE) return;
  const unsigned long now = micros();
  const unsigned long elapsed = now - last_ramp_us;
  if (elapsed < RAMP_UPDATE_US) return;
  last_ramp_us = now;
  const float dt = (float)elapsed * 0.000001f;
  const float change = ACCEL_STEPS_PER_S2 * dt;
  float desired = (float)target_rate_hz;
  if (motion_mode == MotionMode::STOPPING) {
    desired = 0.0f;
  } else if (motion_mode == MotionMode::FINITE) {
    // Brake when remaining distance is the stopping distance to START_RATE_HZ.
    // This yields a triangular ramp for moves too short to reach target speed.
    const float floor_rate = min((float)target_rate_hz, START_RATE_HZ);
    const float stopping_distance =
      (actual_rate_hz * actual_rate_hz - floor_rate * floor_rate) /
      (2.0f * ACCEL_STEPS_PER_S2);
    if ((float)remaining_steps <= stopping_distance + 1.0f) desired = floor_rate;
  }
  if (actual_rate_hz < desired) {
    actual_rate_hz = min(desired, actual_rate_hz + change);
  } else if (actual_rate_hz > desired) {
    actual_rate_hz = max(desired, actual_rate_hz - change);
  }
  if (motion_mode == MotionMode::STOPPING &&
      actual_rate_hz <= min((float)target_rate_hz, START_RATE_HZ)) {
    stopMotion(true);
    return;
  }
  if (actual_rate_hz < 1.0f) actual_rate_hz = 1.0f;
  // Update timing only while pulse is LOW to avoid shortening a HIGH pulse.
  if (!pulse_is_high) updateHalfPeriod();
}
void serviceMotion() {
  if (motion_mode == MotionMode::IDLE) return;
  const unsigned long now = micros();
  if (now - last_edge_us < half_period_us) return;
  last_edge_us = now;  // Delayed loops do not generate catch-up pulse bursts.
  if (!pulse_is_high) {
    digitalWrite(PULSE_PIN, HIGH);
    pulse_is_high = true;
    position_steps += direction_sign;
    if (motion_mode == MotionMode::FINITE) {
      ++completed_move_steps;
      --remaining_steps;
      if (remaining_steps == 0) finish_after_falling_edge = true;
    }
  } else {
    setPulseLow();
    updateHalfPeriod();
    if (finish_after_falling_edge) {
      motion_mode = MotionMode::IDLE;
      target_rate_hz = 0;
      actual_rate_hz = 0.0f;
      finish_after_falling_edge = false;
      idle_since_ms = millis();
      Serial.print(F("DONE "));
      Serial.print(commanded_move_steps);
      Serial.print(F(" POS "));
      Serial.println(position_steps);
    }
  }
}
void serviceProgress() {
  if (motion_mode != MotionMode::FINITE) return;
  unsigned long now = millis();
  if (now - last_progress_ms < PROGRESS_PERIOD_MS) return;
  last_progress_ms = now;
  Serial.print(F("PROGRESS "));
  Serial.print(direction_sign * (long)completed_move_steps);
  Serial.print(F(" OF "));
  Serial.println(commanded_move_steps);
}
void serviceWatchdog() {
  if (motion_mode == MotionMode::IDLE) return;
  if (millis() - last_heartbeat_ms <= WATCHDOG_TIMEOUT_MS) return;
  stopMotion(false);  // Safety: watchdog stop is immediate, NOT ramped.
  disableDriver();
  Serial.println(F("FAULT WATCHDOG"));
}
}  // namespace
void setup() {
  // Preload inactive output level before switching pin to OUTPUT.
  digitalWrite(ENABLE_PIN, DRIVER_DISABLE_LEVEL);
  pinMode(ENABLE_PIN, OUTPUT);
  driver_enabled = false;
  idle_since_ms = millis();
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(PULSE_PIN, OUTPUT);
  digitalWrite(DIRECTION_PIN, LOW);
  setPulseLow();
  Serial.begin(BAUD_RATE);
  delay(300);
  Serial.println(F("READY SYRINGE_STEPPER_RAMP_ENABLE_V3"));
  printStatus();
}
void loop() {
  readSerialCommands();
  serviceWatchdog();
  serviceRamp();
  serviceMotion();
  serviceProgress();
  serviceIdleDisable();
}
