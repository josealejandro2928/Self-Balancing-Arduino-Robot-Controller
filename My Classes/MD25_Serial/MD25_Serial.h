#ifndef MD25_Serial_h
#define MD25_Serial_h

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#include <pins_arduino.h>
#endif

#include <avr/io.h>
#include <avr/interrupt.h>

///////////////////////Class definitions///////////////////
#define START 0x00
#define GET_SPEED1 0x21
#define GET_SPEED2 0x22
#define GET_ENCODER1 0x23
#define GET_ENCODER2 0x24
#define GET_ENCODERS 0x25
#define GET_VOLTS 0x26
#define GET_CURRENT1 0x27
#define GET_CURRENT2 0x28
#define GET_VERSION 0x29
#define GET_ACCELERATION 0x2A
#define GET_MODE 0x2B
#define GET_VI 0x2C
#define SET_SPEED1 0x31
#define SET_SPEED2 0x32
#define SET_ACCELERATIO 0x33
#define SET_MODE 0x34
#define RESET_ENCODERS 0x35
#define DISABLE_REGULATOR 0x36
#define ENABLE_REGULATOR 0x37
#define DISABLE_TIMEOUT 0x38
#define ENEABLE_TIMEOUT 0x39

class MD25_Serial
{
private:
public:
  //Serial1.begin(9600);
  MD25_Serial(int baudrate, int mode, bool autoregulation);
  /*float batteryStatus();
  setSpeedM1(int);
  setSpeedM2(int);
  void setMode(int);
  long getEncoderM1();
  long getEncoderM2();
  float getCurrentM1();
  float getCurrentM2();*/
};

#endif
