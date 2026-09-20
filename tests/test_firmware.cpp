#include "firmware_stub/Arduino.h"
#include <cassert>
#include <iostream>
uint8_t MCUSR=0; uint32_t fake_us=0; int pins[64]={}; SerialMock Serial;
#include "../firmware/syringe_demo/syringe_demo.ino"
void send(const char*s){char b[100];strcpy(b,s);command(b);}
void reset(){halt();faulted=false;fake_us=0;Serial.output.clear();setup();}
int main(){
 reset(); assert(Serial.output.find("READY SYRINGE_DEMO_V1")!=std::string::npos);
 send("MOVE -2 100"); assert(moving && direction==-1);
 fake_us=1000;loop();fake_us=1010;loop();fake_us=11000;loop();fake_us=11010;loop();
 assert(!moving && count==2 && Serial.output.find("DONE 2")!=std::string::npos);
 reset();send("START -1 9");fake_us=1600000;loop();assert(!moving && faulted);
 reset();send("START -1 9");send("STOP");assert(!moving && !faulted && pins[PUL_PIN]==LOW);
 reset();send("MOVE -3001 10");assert(faulted && !moving);
 reset();send("START 0 10");assert(faulted && !moving);
 reset();send("MOVE -2147483648 10");assert(faulted && !moving);
 reset();send("START 1 10 junk");assert(faulted && !moving);
 reset();send("START 1 10");count=MAX_PULSES;pulseHigh=true;fallAt=0;loop();assert(faulted && !moving);
 reset();send("START 1 10");fake_us=120000000;lastPing=millis();loop();assert(faulted && !moving);
 std::cout<<"Firmware logic: 9 scenarios passed (host shim, not AVR timing test)\n";
}
