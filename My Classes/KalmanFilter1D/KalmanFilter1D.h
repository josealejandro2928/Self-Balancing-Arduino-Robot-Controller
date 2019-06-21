#ifndef KalmanFilter1D_h
#define KalmanFilter1D_h

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#include <pins_arduino.h>
#endif

#include <avr/io.h>
#include <avr/interrupt.h>

class KalmanFilter1D
{
  private:
    
    volatile double angle;
    volatile double Q_angle;
    volatile double R_measure;
    volatile double P;
    volatile double K;

  public:
    KalmanFilter1D(double);
    void set_state_covariance(double);
    void set_measurement_covariance(double);
    double get_state(double x, double x_dot, double dt);
};

#endif