#ifndef PID_h
#define PID_h

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#include <pins_arduino.h>
#endif

#include <avr/io.h>
#include <avr/interrupt.h>

class PID
{
private:
  double Kc;
  double Ki;
  double Kd;
  double threshold;
  double max_output;
  double error;
  double l_error;
  double l2_error;
  double l_u_signal;
  double sum_error;
  double u_signal;
  double constraint(double, double, double);
  void update_state();

public:
  PID();
  PID(double, double, double, double, double);
  void reset();
  double classic_compute(double setpoint, double variable, double dt);
  double positional_compute(double setpoint, double variable);
  double velocity_compute(double setpoint, double variable);
  void set_constant(double, double, double);
};

#endif
