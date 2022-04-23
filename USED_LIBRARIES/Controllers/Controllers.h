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

////////// GLOBAL VARIABLES FROM MAIN PROGRAM /////////////////////////////////
extern int GO2GoalMode;
extern float sp_posX;
extern float sp_posY;
extern float velocity;
extern float robotX;
extern float robotY;
extern float robotTheta;
extern float Kc_pos;
extern float Ki_pos;
extern float Kd_pos;
extern float MAX_VEL_PT;
extern float MAX_ANGULAR_VEL_PT;
extern float TRESHOLD_PT;
extern float sp_velocity;
extern float sp_angular_velocity;
extern float Kc_orient;
///////////////////////////////////////////////////////////////////////////////

//////////Global Variables////////////
char modePointTracker = 'F';
float last_inclination = 0.0;
float sum_error_inclination = 0.0;

float last_vel = 0.0;
float sum_error_vel = 0.0;

float last_angular_vel = 0.0;
float sum_error_angular_vel = 0.0;

float last_error_pos = 0.0;
float sum_error_pos = 0.0;
//////////////////////////////////////
///////////////UTILS FUNCTIONS //////////////////////////////////
float constrainAngleToPI(float angle)
{
    if (angle > 0.0)
    {
        angle = ((angle >= (2 * PI))) ? (angle - (2 * PI)) : angle;
    }
    else if (angle < 0.0)
    {
        angle = (angle >= -(2 * PI)) ? (angle + (2 * PI)) : -1.0 * (angle + (2 * PI));
    }
    return angle;
}
////////////////////////////////////////////////////////////////

float inclinationPID(float setpoint, float inclination, float max_output, float dt, float &Kc, float &Ki, float &Kd)
{
    //// THE ROBOT IS FALLING DOWN WE SHOULD TURN OFF THE MOTORS ///
    if (abs(inclination) > 65)
    {
        last_inclination = 0.0;
        sum_error_inclination = 0.0;
        return 0.0;
    }
    float error = setpoint - inclination;
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

float velocityPID(float setpoint, float vel, float max_output, float dt, float &Kc, float &Ki, float &Kd, float inclination)
{
    //// THE ROBOT IS FALLING DOWN WE SHOULD TURN OFF THE MOTORS ///
    if (abs(inclination) > 65)
    {
        last_vel = 0.0;
        sum_error_vel = 0.0;
        return 0.0;
    }

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

float angularVelocityPID(float setpoint, float angular_vel, float max_output, float dt, float &Kc, float &Ki, float &Kd, float inclination)
{
    //// THE ROBOT IS FALLING DOWN WE SHOULD TURN OFF THE MOTORS ///
    if (abs(inclination) > 65)
    {
        last_angular_vel = 0.0;
        sum_error_angular_vel = 0.0;
        return 0.0;
    }

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

//// POINT TRACKER ALGORITMS ///
void point_traker_method(float dt)
{
    float theta_forward = robotTheta;
    float theta_backward = robotTheta;
    float posError = 0.0;
    float orientError = 0.0;
    float u_pos = 0.0;
    float u_orient = 0.0;

    if (theta_forward > PI && theta_forward < (2 * PI))
    {
        theta_forward = theta_forward - (2 * PI);
    }
    theta_backward = theta_forward - PI;
    float dist_to_goal = sqrt(pow((sp_posX - robotX), 2.0) + pow((sp_posY - robotY), 2.0));
    float theta_goal = atan2(sp_posY - robotY, sp_posX - robotX);

    //////////Cambio de modos///////////
    float backward_boundery = constrainAngleToPI(theta_backward) - constrainAngleToPI(theta_goal);
    if ((abs(backward_boundery) <= 1.57) && dist_to_goal <= 1.25)
    {
        modePointTracker = 'B';
    }
    else
    {
        modePointTracker = 'F';
    }

    ////////////////////////////////////////////

    if (modePointTracker == 'F')
    {
        //////// Orientation Controller/////
        float dist1 = constrainAngleToPI(theta_goal) - constrainAngleToPI(theta_forward);
        float dist2 = theta_goal - theta_forward;
        if (abs(dist1) < abs(dist2))
        {
            orientError = dist1;
        }
        else if (abs(dist1) > abs(dist2))
        {
            orientError = dist2;
        }
        else
        {
            orientError = dist2;
        }

        /////////Orientacion Control//////////
        u_orient = constrain(Kc_orient * orientError, -1.0 * MAX_ANGULAR_VEL_PT, MAX_ANGULAR_VEL_PT);

        //////// Position Controller/////
        posError = dist_to_goal;
        sum_error_pos = Ki_pos * (sum_error_pos + posError) * dt;
        sum_error_pos = constrain(sum_error_pos, -1.0 * MAX_VEL_PT, MAX_VEL_PT);
        float I_term = sum_error_pos;
        float D_term = Kd_pos * (posError - last_error_pos) / dt;
        u_pos = constrain(Kc_pos * posError + I_term + D_term, -1.0 * MAX_VEL_PT, MAX_VEL_PT);
        last_error_pos = posError;
    }
    else
    {
        float dist1 = constrainAngleToPI(theta_goal) - constrainAngleToPI(theta_backward);
        float dist2 = theta_goal - theta_backward;
        if (abs(dist1) < abs(dist2))
        {
            orientError = dist1;
        }
        else if (abs(dist1) > abs(dist2))
        {
            orientError = dist2;
        }
        else
        {
            orientError = dist1;
        }
        /////////Orientacion Control//////////
        u_orient = constrain(Kc_orient * orientError, -1.0 * MAX_ANGULAR_VEL_PT, MAX_ANGULAR_VEL_PT);

        //////// Position Controller/////
        posError = dist_to_goal;
        sum_error_pos = Ki_pos * (sum_error_pos + posError) * dt;
        sum_error_pos = constrain(sum_error_pos, -1.0 * (MAX_VEL_PT - 0.05), (MAX_VEL_PT - 0.05));
        float I_term = sum_error_pos;
        float D_term = Kd_pos * (posError - last_error_pos) / dt;
        u_pos = -1.0 * constrain(Kc_pos * posError + I_term + D_term, -1.0 * (MAX_VEL_PT - 0.05), (MAX_VEL_PT - 0.05));
        last_error_pos = posError;
    }

    if (abs(posError) <= TRESHOLD_PT)
    {
        u_pos = 0.0;
        u_orient = 0.0;
    }

    sp_velocity = u_pos;
    sp_angular_velocity = u_orient;
}

//////////////////////////////////////////////////////////////////////////////////////////////
#endif
