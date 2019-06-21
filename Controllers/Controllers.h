#ifndef Controllers_h
#define Controllers_h

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#include <pins_arduino.h>
#endif

#include <avr/io.h>
#include <avr/interrupt.h>

//////////Global Variables////////////
float last_inclination = 0.0;
float sum_error_inclination = 0.0;

float last_vel = 0.0;
float sum_error_vel = 0.0;

float last_angular_vel = 0.0;
float sum_error_angular_vel = 0.0;
//////////////////////////////////////

float inclinationPID(float setpoint, float inclination, float max_output, float dt, float &Kc, float &Ki, float &Kd)
{
    float error = setpoint - inclination;
    if (abs(error) >= 60)
    {
        sum_error_inclination = 0.0;
        last_inclination = 0.0;
        return 0.0;
    }
    sum_error_inclination += (Ki * error) * dt;
    sum_error_inclination = constrain(sum_error_inclination, -1.0 * max_output, max_output);
    float P = Kc * error;
    float I = sum_error_inclination;
    float D = -1.0 * (Kd * ((inclination - last_inclination) / dt));
    float output = P + I + D;
    output = constrain(output, -1.0 * max_output, max_output);
    last_inclination = inclination;
    return output;
}

float velocityPID(float setpoint, float vel, float max_output, float dt, float &Kc, float &Ki, float &Kd)
{
    float error = setpoint - vel;

    sum_error_vel += (Ki * error) * dt;
    sum_error_vel = constrain(sum_error_vel, -1.0 * max_output, max_output);
    float P = Kc * error;
    float I = sum_error_vel;
    float D = -1.0 * (Kd * ((vel - last_vel) / dt));
    float output = P + I + D;
    output = constrain(output, -1.0 * max_output, max_output);
    last_vel = vel;
    return output;
}

float angularVelocityPID(float setpoint, float angular_vel, float max_output, float dt, float &Kc, float &Ki, float &Kd)
{
    float error = setpoint - angular_vel;
    sum_error_angular_vel += (Ki * error) * dt;
    sum_error_angular_vel = constrain(sum_error_angular_vel, -1.0 * max_output, max_output);
    float P = Kc * error;
    float I = sum_error_angular_vel;
    float D = -1.0 * (Kd * ((angular_vel - last_angular_vel) / dt));
    float output = P + I + D;
    output = constrain(output, -1.0 * max_output, max_output);
    last_angular_vel = angular_vel;
    return output;
}

#endif
