// Arduino Mega 2560. Protocol used by syringe_controller/serial_backend.py.
// Positive steps set DIR HIGH; verify physical direction at low speed unloaded.
#include <Arduino.h>
#include <avr/wdt.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

const uint8_t PUL_PIN=28, DIR_PIN=27, ENA_PIN=25;
// ENA was disconnected in the working lab setup. Only enable after polarity test.
const bool USE_ENABLE=false;
const uint8_t ENABLE_ACTIVE=LOW;
// Optional normally-closed switches: pin to GND when healthy, open = trip.
const bool USE_INTERLOCKS=false;
const uint8_t ESTOP_PIN=30, LOWER_LIMIT_PIN=32, UPPER_LIMIT_PIN=34;
const int8_t EXTRUSION_DIRECTION=-1;
const uint32_t WATCHDOG_MS=1500, MAX_RUN_MS=120000;
const uint32_t MAX_PULSES=3000, MAX_RATE=5000, PULSE_US=10;

bool moving=false, finiteMove=false, pulseHigh=false, faulted=false;
int8_t direction=1;
uint32_t target=0, count=0, intervalUs=0, nextPulse=0, fallAt=0;
uint32_t lastPing=0, started=0, lastReport=0;
char buffer[80]; uint8_t used=0; bool overflow=false;

void halt() {
  moving=false; pulseHigh=false; digitalWrite(PUL_PIN,LOW);
  if(USE_ENABLE) digitalWrite(ENA_PIN, ENABLE_ACTIVE==HIGH ? LOW : HIGH);
}
void trip(const char *reason) {
  halt(); faulted=true; Serial.print("FAULT "); Serial.println(reason);
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
  char *cmd=strtok(s," \r"), *a=strtok(NULL," \r"), *b=strtok(NULL," \r");
  char *extra=strtok(NULL," \r");
  if(!cmd) return;
  if(!strcmp(cmd,"STOP") && !a) {
    halt(); Serial.println("ACK STOP"); Serial.print("STOPPED "); Serial.println(count); return;
  }
  if(!strcmp(cmd,"PING") && !a) { lastPing=millis(); Serial.println("PONG"); return; }
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
  finiteMove=isMove; intervalUs=1000000UL/(uint32_t)rate;
  digitalWrite(DIR_PIN,direction>0 ? HIGH : LOW);
  if(USE_ENABLE) digitalWrite(ENA_PIN,ENABLE_ACTIVE);
  started=lastPing=lastReport=millis(); nextPulse=micros()+1000;
  moving=true;
  Serial.print("ACK "); Serial.println(cmd);
}
void setup() {
  // This is a reset watchdog, separate from the host heartbeat watchdog.
  MCUSR=0; wdt_disable();
  pinMode(PUL_PIN,OUTPUT); pinMode(DIR_PIN,OUTPUT); pinMode(ENA_PIN,OUTPUT);
  pinMode(ESTOP_PIN,INPUT_PULLUP); pinMode(LOWER_LIMIT_PIN,INPUT_PULLUP);
  pinMode(UPPER_LIMIT_PIN,INPUT_PULLUP); halt();
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
  if(!moving) return;
  if(interlock()) { trip("INTERLOCK"); return; }
  if(ms-lastPing>WATCHDOG_MS) { trip("HOST_TIMEOUT"); return; }
  if(ms-started>=MAX_RUN_MS) { trip("RUN_TIME_LIMIT"); return; }
  if(pulseHigh && (int32_t)(now-fallAt)>=0) {
    digitalWrite(PUL_PIN,LOW); pulseHigh=false;
    if(count>=target) {
      halt();
      if(finiteMove) { Serial.print("DONE "); Serial.println(count); }
      else trip("PULSE_BUDGET");
      return;
    }
  }
  if(!pulseHigh && (int32_t)(now-nextPulse)>=0) {
    digitalWrite(PUL_PIN,HIGH); pulseHigh=true; ++count;
    fallAt=now+PULSE_US; nextPulse=now+intervalUs; // never catch up missed pulses
  }
  if(ms-lastReport>=250 && Serial.availableForWrite()>=32) {
    lastReport=ms; Serial.print("PROGRESS "); Serial.println(count);
  }
}
