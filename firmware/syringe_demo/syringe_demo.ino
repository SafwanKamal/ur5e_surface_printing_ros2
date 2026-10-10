// Arduino Mega 2560. Protocol used by syringe_controller/serial_backend.py.
// Positive steps set DIR HIGH; verify physical direction at low speed unloaded.
#include <Arduino.h>
#include <avr/wdt.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <ctype.h>

const uint8_t PUL_PIN=28, DIR_PIN=27, ENA_PIN=25;
// ENA+ -> Mega 25, ENA- -> signal GND. Verify polarity on the actual driver.
const bool USE_ENABLE=true;
const uint8_t ENABLE_ACTIVE=LOW;
// Optional normally-closed switches: pin to GND when healthy, open = trip.
const bool USE_INTERLOCKS=false;
const uint8_t ESTOP_PIN=30, LOWER_LIMIT_PIN=32, UPPER_LIMIT_PIN=34;
const int8_t EXTRUSION_DIRECTION=-1;
const uint32_t WATCHDOG_MS=1500, MAX_RUN_MS=120000;
// Units are driver pulses (including microsteps), not full motor steps.
const float START_RATE_HZ=100.0f, ACCEL_PULSES_S2=1000.0f;
const uint32_t ENABLE_SETTLE_US=1000, IDLE_DISABLE_MS=300;
const uint32_t MAX_PULSES=3000, MAX_RATE=5000, PULSE_US=10;

bool moving=false, finiteMove=false, pulseHigh=false, faulted=false;
int8_t direction=1;
uint32_t target=0, count=0, intervalUs=0, nextPulse=0, fallAt=0;
uint32_t lastPing=0, started=0, lastReport=0;
bool watchdogEnabled=true, driverEnabled=false, idleDisablePending=false;
bool rampStopping=false;
uint32_t watchdogTimeoutMs=WATCHDOG_MS, idleSince=0, lastRise=0;
uint32_t requestedRate=0;
float currentRate=0, initialRate=0;
char buffer[80]; uint8_t used=0; bool overflow=false;

void setDriver(bool enabled) {
  driverEnabled=enabled;
  if(USE_ENABLE) digitalWrite(ENA_PIN,enabled ? ENABLE_ACTIVE :
    (ENABLE_ACTIVE==HIGH ? LOW : HIGH));
}
void halt(bool immediate) {
  moving=false; pulseHigh=false; rampStopping=false; currentRate=0;
  digitalWrite(PUL_PIN,LOW);
  idleDisablePending=!immediate; idleSince=millis();
  if(immediate) setDriver(false);
}
// Called after each falling edge. Distance-domain ramps do not accelerate
// while pulse generation is delayed, and preserve the exact finite pulse count.
void updateInterval() {
  float rateSquared;
  if(rampStopping) {
    rateSquared=currentRate*currentRate-2.0f*ACCEL_PULSES_S2;
    if(rateSquared<initialRate*initialRate) rateSquared=initialRate*initialRate;
  } else {
    uint32_t distance=count;
    if(finiteMove && target-1-count<distance) distance=target-1-count;
    rateSquared=initialRate*initialRate+2.0f*ACCEL_PULSES_S2*distance;
  }
  currentRate=sqrtf(rateSquared);
  if(currentRate>requestedRate) currentRate=requestedRate;
  intervalUs=(uint32_t)ceilf(1000000.0f/currentRate);
  nextPulse=lastRise+intervalUs;
}
void trip(const char *reason) {
  halt(true); faulted=true; Serial.print("FAULT "); Serial.println(reason);
}
bool number(const char *s, long &value) {
  if(!s || !*s) return false;
  char *end; errno=0; value=strtol(s,&end,10);
  return !errno && *end=='\0';
}
bool interlock() {
  return USE_INTERLOCKS && (digitalRead(ESTOP_PIN)==HIGH ||
    (direction==EXTRUSION_DIRECTION ? digitalRead(LOWER_LIMIT_PIN)==HIGH
                                    : digitalRead(UPPER_LIMIT_PIN)==HIGH));
}
void command(char *s) {
  for(char *p=s; *p; ++p) *p=(char)toupper((unsigned char)*p);
  char *cmd=strtok(s," \t\r"), *a=strtok(NULL," \t\r"), *b=strtok(NULL," \t\r");
  char *extra=strtok(NULL," \t\r");
  if(!cmd) return;
  if((!strcmp(cmd,"STOP") || !strcmp(cmd,"ESTOP")) && !a) {
    halt(true); Serial.print("ACK "); Serial.println(cmd); Serial.print("STOPPED "); Serial.println(count); return;
  }
  if(!strcmp(cmd,"PING") && !a) { lastPing=millis(); Serial.println("PONG"); return; }
  if(!strcmp(cmd,"RAMP_STOP") && !a) {
    if(moving) rampStopping=true;
    Serial.println("ACK RAMP_STOP");
    if(!moving) { Serial.print("STOPPED "); Serial.println(count); }
    return;
  }
  if(!strcmp(cmd,"STATUS") && !a) {
    Serial.print("STATE "); Serial.print(moving ? "MOVING" : "IDLE");
    Serial.print(" COUNT "); Serial.print(count);
    Serial.print(" RATE "); Serial.print((uint32_t)currentRate);
    Serial.print(" TARGET "); Serial.print(requestedRate);
    Serial.print(" DRIVER "); Serial.print(driverEnabled ? "ENABLED" : "DISABLED");
    Serial.print(" FAULT "); Serial.println(faulted ? "LATCHED" : "NONE"); return;
  }
  if(!strcmp(cmd,"WATCHDOG")) {
    long timeout;
    if(a && !b && !extra && (!strcmp(a,"ON") || !strcmp(a,"OFF"))) {
      watchdogEnabled=!strcmp(a,"ON"); lastPing=millis();
      Serial.print("ACK WATCHDOG "); Serial.println(a); return;
    }
    if(a && !strcmp(a,"STATUS") && !b && !extra) {
      Serial.print("WATCHDOG "); Serial.print(watchdogEnabled ? "ON" : "OFF");
      Serial.print(" TIMEOUT "); Serial.println(watchdogTimeoutMs); return;
    }
    if(a && !strcmp(a,"TIMEOUT") && !extra && number(b,timeout) &&
       timeout>=100 && timeout<=60000) {
      watchdogTimeoutMs=(uint32_t)timeout; lastPing=millis();
      Serial.println("ACK WATCHDOG TIMEOUT"); return;
    }
    trip("BAD_COMMAND"); return;
  }
  long amount, rate;
  bool isMove=!strcmp(cmd,"MOVE"), isStart=!strcmp(cmd,"START");
  if((!isMove && !isStart) || extra || !number(a,amount) || !number(b,rate) ||
     amount==0 || amount==(-2147483647L-1L) || rate<1 || rate>(long)MAX_RATE ||
     (isStart && amount!=1 && amount!=-1) ||
     (isMove && (uint32_t)labs(amount)>MAX_PULSES)) {
    trip("BAD_COMMAND"); return;
  }
  if(faulted) { Serial.println("ERROR LATCHED_RESET_REQUIRED"); return; }
  if(moving) { trip("BUSY"); return; }
  direction=amount>0 ? 1 : -1;
  if(interlock()) { trip("INTERLOCK"); return; }
  count=0; target=isMove ? (uint32_t)labs(amount) : MAX_PULSES;
  finiteMove=isMove; rampStopping=false; requestedRate=(uint32_t)rate;
  initialRate=rate<START_RATE_HZ ? (float)rate : START_RATE_HZ;
  currentRate=initialRate; intervalUs=(uint32_t)ceilf(1000000.0f/currentRate);
  digitalWrite(DIR_PIN,direction>0 ? HIGH : LOW);
  idleDisablePending=false; setDriver(true);
  started=lastPing=lastReport=millis(); nextPulse=micros()+ENABLE_SETTLE_US;
  moving=true;
  Serial.print("ACK "); Serial.println(cmd);
}
void setup() {
  // This is a reset watchdog, separate from the host heartbeat watchdog.
  MCUSR=0; wdt_disable();
  watchdogEnabled=true; watchdogTimeoutMs=WATCHDOG_MS;
  faulted=false; count=0; target=0; requestedRate=0;
  // Set output latch before making ENA an output to avoid an enable glitch.
  digitalWrite(ENA_PIN,ENABLE_ACTIVE==HIGH ? LOW : HIGH);
  pinMode(PUL_PIN,OUTPUT); pinMode(DIR_PIN,OUTPUT); pinMode(ENA_PIN,OUTPUT);
  pinMode(ESTOP_PIN,INPUT_PULLUP); pinMode(LOWER_LIMIT_PIN,INPUT_PULLUP);
  pinMode(UPPER_LIMIT_PIN,INPUT_PULLUP); halt(true);
  Serial.begin(115200); Serial.println("READY SYRINGE_DEMO_V1");
  wdt_enable(WDTO_2S);
}
void loop() {
  wdt_reset();
  // Bounded parsing: serial flood must not starve pulse/stop/watchdog checks.
  for(uint8_t n=0; n<32 && Serial.available(); ++n) {
    char c=(char)Serial.read();
    if(c=='\n') {
      if(!overflow) { buffer[used]='\0'; command(buffer); }
      used=0; overflow=false;
    } else if(used<sizeof(buffer)-1 && !overflow) buffer[used++]=c;
    else if(!overflow) { overflow=true; trip("LINE_TOO_LONG"); }
  }
  uint32_t now=micros(), ms=millis();
  if(!moving) {
    if(idleDisablePending && ms-idleSince>=IDLE_DISABLE_MS) {
      idleDisablePending=false; setDriver(false);
    }
    return;
  }
  if(interlock()) { trip("INTERLOCK"); return; }
  if(watchdogEnabled && ms-lastPing>watchdogTimeoutMs) { trip("HOST_TIMEOUT"); return; }
  if(ms-started>=MAX_RUN_MS) { trip("RUN_TIME_LIMIT"); return; }
  if(pulseHigh && (int32_t)(now-fallAt)>=0) {
    digitalWrite(PUL_PIN,LOW); pulseHigh=false;
    if(count>=target) {
      halt(false);
      if(finiteMove) { Serial.print("DONE "); Serial.println(count); }
      else trip("PULSE_BUDGET");
      return;
    }
    if(rampStopping && currentRate<=initialRate) {
      halt(false); Serial.print("STOPPED "); Serial.println(count); return;
    }
    updateInterval();
  }
  if(!pulseHigh && (int32_t)(now-nextPulse)>=0) {
    digitalWrite(PUL_PIN,HIGH); pulseHigh=true; ++count;
    lastRise=now; fallAt=now+PULSE_US; nextPulse=now+intervalUs; // never catch up missed pulses
  }
  if(ms-lastReport>=250 && Serial.availableForWrite()>=32) {
    lastReport=ms; Serial.print("PROGRESS "); Serial.println(count);
  }
}
