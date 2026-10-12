#include "firmware_stub/Arduino.h"
#include <cassert>
#include <iostream>
#include <cstring>
uint8_t MCUSR=0; uint32_t fake_us=0; int pins[64]={}; SerialMock Serial;
#include "../firmware/syringe_stepper_ramped_enable/syringe_stepper_ramped_enable.ino"
void send(const char* text) {char line[100];strcpy(line,text);handleCommand(line);}
void reset() {fake_us=0;motion_mode=MotionMode::IDLE;position_steps=0;
  driver_enabled=false;Serial.output.clear();setup();}
int main() {
  reset();assert(Serial.output.find("READY SYRINGE_STEPPER_RAMP_ENABLE_V3")!=std::string::npos);
  for(long steps : {-2000L,4000L}) {
    reset();std::string cmd="MOVE "+std::to_string(steps)+" 200";send(cmd.c_str());
    assert(actual_rate_hz==100 && driver_enabled && pins[ENABLE_PIN]==LOW);
    unsigned long heartbeat=millis();
    while(motion_mode!=MotionMode::IDLE && fake_us<60000000) {
      fake_us+=100;loop();
      if(millis()-heartbeat>=1000) {send("PING");heartbeat=millis();}
    }
    assert(motion_mode==MotionMode::IDLE);
    assert(completed_move_steps==(unsigned long)labs(steps) && position_steps==steps);
    assert(Serial.output.find("DONE "+std::to_string(steps)+" POS ")!=std::string::npos);
    fake_us+=300000;loop();assert(!driver_enabled);
  }
  reset();send("START -1 200");fake_us+=100000;serviceRamp();
  assert(actual_rate_hz==200);
  send("STOP");assert(motion_mode==MotionMode::STOPPING);
  assert(Serial.output.find("ACK STOPPING")!=std::string::npos);
  send("ESTOP");assert(motion_mode==MotionMode::IDLE && !driver_enabled);
  assert(Serial.output.find("ACK ESTOP")!=std::string::npos);
  reset();send("START -1 200");fake_us+=21000000;loop();
  assert(motion_mode==MotionMode::IDLE && !driver_enabled);
  assert(Serial.output.find("FAULT WATCHDOG")!=std::string::npos);
  reset();send("MOVE 2000001 200");assert(motion_mode==MotionMode::FINITE);send("ESTOP");
  reset();send("MOVE 2147483648 200");assert(motion_mode==MotionMode::IDLE);
  std::cout<<"Working V3 counts, ramp, enable, STOP/ESTOP and watchdog checks passed (host shim)\n";
}
