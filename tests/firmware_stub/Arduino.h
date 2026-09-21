#pragma once
#include <stdint.h>
#include <string>
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT_PULLUP 2
extern uint8_t MCUSR;
extern uint32_t fake_us;
extern int pins[64];
inline uint32_t micros(){return fake_us;}
inline uint32_t millis(){return fake_us/1000;}
inline void pinMode(int,int){}
inline void digitalWrite(int p,int v){pins[p]=v;}
inline int digitalRead(int p){return pins[p];}
struct SerialMock {
 std::string output;
 void begin(int){}
 int available(){return 0;}
 int availableForWrite(){return 64;}
 int read(){return 0;}
 void print(const char*s){output+=s;}
 void print(uint32_t v){output+=std::to_string(v);}
 void println(const char*s){print(s);output+='\n';}
 void println(uint32_t v){print(v);output+='\n';}
};
extern SerialMock Serial;
