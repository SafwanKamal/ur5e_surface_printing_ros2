#include "firmware_stub/Arduino.h"
#include <cassert>
#include <iostream>
#include <vector>
#include <cmath>
uint8_t MCUSR=0; uint32_t fake_us=0; int pins[64]={}; SerialMock Serial;
#include "../firmware/syringe_demo/syringe_demo.ino"
void send(const char*s){char b[100];strcpy(b,s);command(b);}
void reset(){halt(true);faulted=false;fake_us=0;Serial.output.clear();setup();}
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
 // Exact finite counts and symmetric triangular/plateau profiles, both signs.
 for(long amount : {1L,-2L,25L,-1000L,3000L}) {
   for(long rate : {1L,50L,100L,500L,5000L}) {
     reset(); send("WATCHDOG OFF");
     std::string cmd="MOVE "+std::to_string(amount)+" "+std::to_string(rate);
     send(cmd.c_str());
     std::vector<float> speeds;
     while(moving && fake_us<100000000) {
       speeds.push_back(currentRate);
       fake_us=nextPulse; loop();
       assert(pins[PUL_PIN]==HIGH && pulseHigh);
       fake_us=fallAt; loop();
     }
     // Long low-rate commands still obey the runtime cap: test a short subset.
     if((uint32_t)labs(amount)*(1000000UL/(uint32_t)rate)>90000000UL) {
       send("STOP"); continue;
     }
     assert(!moving && !faulted && count==(uint32_t)labs(amount));
     assert(speeds.size()==count && currentRate==0 && pins[PUL_PIN]==LOW);
     for(size_t i=0;i<speeds.size();++i) {
       assert(speeds[i]<=rate);
       assert(fabs(speeds[i]-speeds[speeds.size()-1-i])<0.01f);
     }
     assert(driverEnabled && idleDisablePending);
     fake_us+=IDLE_DISABLE_MS*1000UL; loop();
     assert(!driverEnabled && pins[ENA_PIN]!=ENABLE_ACTIVE);
   }
 }
 reset(); send("WATCHDOG OFF"); send("START 1 5000");
 // Reach > starting speed, then brake without exceeding the pulse budget.
 for(int i=0;i<250;++i) { fake_us=nextPulse;loop();fake_us=fallAt;loop(); }
 assert(currentRate>500); send("RAMP_STOP");
 float previous=currentRate;
 while(moving) {
   fake_us=nextPulse;loop();fake_us=fallAt;loop();
   assert(currentRate<=previous);previous=currentRate;
 }
 assert(!faulted && count>250 && Serial.output.find("STOPPED ")!=std::string::npos);
 reset();send("WATCHDOG OFF");send("START 1 10");
 fake_us=1600000;loop();assert(moving && !faulted);
 send("WATCHDOG ON");fake_us+=1600000;loop();assert(faulted && !moving && !driverEnabled);
 // Testing overrides never disable run caps, and reset restores defaults.
 reset();send("WATCHDOG OFF");send("START 1 10");
 fake_us=MAX_RUN_MS*1000UL;loop();assert(faulted && !moving);
 reset();send("WATCHDOG TIMEOUT 5000");send("START 1 10");
 fake_us=2000000;loop();assert(moving);
 fake_us=5001000;loop();assert(faulted && !moving);
 reset();assert(watchdogEnabled && watchdogTimeoutMs==WATCHDOG_MS);
 send("watchdog off");assert(!watchdogEnabled);
 send("WATCHDOG TIMEOUT 99");assert(faulted);
 send("WATCHDOG ON");send("MOVE 1 100");assert(faulted && !moving);
 reset();send("START 1 100");send("ESTOP");
 assert(!moving && !driverEnabled && Serial.output.find("ACK ESTOP")!=std::string::npos);
 reset();send("STATUS");assert(Serial.output.find("DRIVER DISABLED")!=std::string::npos);
 // MOV is an alias; both finite and continuous commands ramp from 100 Hz.
 for(const char* cmd : {"MOV -200 200", "MOVE -200 200", "START -1 200"}) {
   reset();send("WATCHDOG OFF");send(cmd);
   assert(moving && currentRate==100 && requestedRate==200);
   for(int i=0;i<15;++i) {fake_us=nextPulse;loop();fake_us=fallAt;loop();}
   assert(currentRate==200 && !faulted);
   send("STOP");assert(!moving && !driverEnabled);
 }
 std::cout<<"Firmware counts, ramps, stops, enable and watchdog checks passed "
          <<"(host shim, not AVR timing test)\n";
}
