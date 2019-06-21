#ifndef ComplementaryFilter_h
#define ComplementaryFilter_h

/// Esta libreria fue desarrollada por Jose Alejandro Concepcion Alvarez///
///Es sencillamente para aplicar el filtro complemento para obtener la inclinacion del selfbalancing////
#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#include <pins_arduino.h>
#endif

#include <avr/io.h>
#include <avr/interrupt.h>

class ComplementaryFilter
{
  private:
    float A;
    float B;
    float variable;
  public:
    ComplementaryFilter();
    ComplementaryFilter(float A,float B,float x);
    float get_measurement(float x , float x_dot,float dt);
    void set_constants(float A,float B);
};

#endif