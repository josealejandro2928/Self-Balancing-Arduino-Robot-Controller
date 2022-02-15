#ifndef KalmanFilterPro_h
#define KalmanFilterPro_h

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#include <pins_arduino.h>
#endif

#include <avr/io.h>
#include <avr/interrupt.h>

class KalmanFilterPro
{
  private:
    
    float angle;
    float Q_angle;
    float Q_gyro_bias;
    float R_measure;
    float P[2][2];
    float K[2];
    float bias;
    float rate;

  public:
    KalmanFilterPro(float);
    void set_state_covariance(float,float);
    void set_measurement_covariance(float);
    float get_state(float x, float x_dot, float dt);
};

#endif