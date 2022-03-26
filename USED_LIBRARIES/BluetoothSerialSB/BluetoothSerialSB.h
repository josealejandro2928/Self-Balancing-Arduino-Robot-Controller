#ifndef BluetoothSerialSB_h
#define BluetoothSerialSB_h

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#include <pins_arduino.h>
#endif

#include <avr/io.h>
#include <avr/interrupt.h>

////////// GLOBAL VARIABLES FROM MAIN PROGRAM ////////////////////////////
extern float sp_velocity;
extern float sp_angular_velocity;
extern int GO2GoalMode;
extern float sp_posX;
extern float sp_posY;
extern float velocity;
extern float angular_velocity;
extern float inclination;
extern float robotX;
extern float robotY;
extern float robotTheta;
extern volatile float charge;
extern float Kc_i;
extern float Ki_i;
extern float Kd_i;
extern float Kc_v;
extern float Ki_v;
extern float Kd_v;
extern float Kc_w;
extern float Ki_w;
extern float Kd_w;
extern float MIN_ABS_SPEED;
////////////////////Bluetooth Protocol////////////////////////////////////
#define COMMAND_SETPOINT_SPEEDS 'A'
#define COMMAND_GETSTATE 'B'
#define SET_PID_K_INCLINATION 'C'
#define GET_PID_K_INCLINATION 'D'
#define SET_PID_K_VELOCITY 'E'
#define GET_PID_K_VELOCITY 'F'
#define SET_PID_K_ANGULAR_VELOCITY 'G'
#define GET_PID_K_ANGULAR_VELOCITY 'H'
#define POINT_TRACKER_MODE 'P'
#define STOP_POINT_TRACKER_MODE 'S'
#define RESET_DYNAMICAL_STATE 'R'
#define GET_BATTERY_STATE 'T'
#define GET_CONSTANT_PARAMETERS 'J'
#define SET_CONSTANT_PARAMETERS 'K'
unsigned char bufferData[4] = {0, 0, 0, 0};
/////////////////////////////////////////////////////////////////////////

float bytesToFloat(unsigned char data[4])
{
    float output;
    *((unsigned char *)(&output) + 3) = data[3];
    *((unsigned char *)(&output) + 2) = data[2];
    *((unsigned char *)(&output) + 1) = data[1];
    *((unsigned char *)(&output) + 0) = data[0];
    return output;
}

void bluetooth_serial()
{
    if (Serial3.available() > 0)
    {
        char command = Serial3.read();
        Serial.println(command);

        if (command == COMMAND_SETPOINT_SPEEDS)
        {
            while (Serial3.available() < 4)
                ;
            Serial3.readBytes(bufferData, 4);
            sp_velocity = bytesToFloat(bufferData);
            while (Serial3.available() < 4)
                ;
            Serial3.readBytes(bufferData, 4);
            sp_angular_velocity = bytesToFloat(bufferData);
            GO2GoalMode = 0;
        }
        else if (command == POINT_TRACKER_MODE)
        {
            while (Serial3.available() < 4)
                ;
            Serial3.readBytes(bufferData, 4);
            sp_posX = bytesToFloat(bufferData);
            while (Serial3.available() < 4)
                ;
            Serial3.readBytes(bufferData, 4);
            sp_posY = bytesToFloat(bufferData);
            GO2GoalMode = 1;
        }
        else if (command == COMMAND_GETSTATE)
        {
            Serial3.println(velocity);
            Serial3.println(angular_velocity);
            Serial3.println(inclination);
            Serial3.println(robotX);
            Serial3.println(robotY);
            Serial3.println(degrees(robotTheta));
            Serial3.println(charge);
        }
        else if (command == SET_PID_K_INCLINATION)
        {
            while (Serial3.available() < 4)
                ;
            Serial3.readBytes(bufferData, 4);
            Kc_i = bytesToFloat(bufferData);

            while (Serial3.available() < 4)
                ;
            Serial3.readBytes(bufferData, 4);
            Ki_i = bytesToFloat(bufferData);

            while (Serial3.available() < 4)
                ;
            Serial3.readBytes(bufferData, 4);
            Kd_i = bytesToFloat(bufferData);
        }
        else if (command == GET_PID_K_INCLINATION)
        {
            Serial3.println(Kc_i);
            Serial3.println(Ki_i);
            Serial3.println(Kd_i);
        }
        else if (command == SET_PID_K_VELOCITY)
        {
            while (Serial3.available() < 4)
                ;
            Serial3.readBytes(bufferData, 4);
            Kc_v = bytesToFloat(bufferData);
            while (Serial3.available() < 4)
                ;
            Serial3.readBytes(bufferData, 4);
            Ki_v = bytesToFloat(bufferData);
            while (Serial3.available() < 4)
                ;
            Serial3.readBytes(bufferData, 4);
            Kd_v = bytesToFloat(bufferData);
        }
        else if (command == GET_PID_K_VELOCITY)
        {
            Serial3.println(Kc_v);
            Serial3.println(Ki_v);
            Serial3.println(Kd_v);
        }
        else if (command == SET_PID_K_ANGULAR_VELOCITY)
        {
            while (Serial3.available() < 4)
                ;
            Serial3.readBytes(bufferData, 4);
            Kc_w = bytesToFloat(bufferData);
            while (Serial3.available() < 4)
                ;
            Serial3.readBytes(bufferData, 4);
            Ki_w = bytesToFloat(bufferData);
            while (Serial3.available() < 4)
                ;
            Serial3.readBytes(bufferData, 4);
            Kd_w = bytesToFloat(bufferData);
        }
        else if (command == GET_PID_K_ANGULAR_VELOCITY)
        {
            Serial3.println(Kc_w);
            Serial3.println(Ki_w);
            Serial3.println(Kd_w);
        }
        else if (command == RESET_DYNAMICAL_STATE)
        {
            robotX = 0.0;
            robotY = 0.0;
            robotTheta = 0.0;
            GO2GoalMode = 0;
            sp_velocity = 0.0;
            sp_angular_velocity = 0.0;
        }
        else if (command == GET_BATTERY_STATE)
        {
            Serial3.println(charge);
        }
        else if (command == GET_CONSTANT_PARAMETERS)
        {
            Serial3.println(MIN_ABS_SPEED);
        }
        else if (command == SET_CONSTANT_PARAMETERS)
        {
            while (Serial3.available() < 4)
                ;
            Serial3.readBytes(bufferData, 4);
            MIN_ABS_SPEED = bytesToFloat(bufferData);
        }
    }
}

//////////////////////////////////////////////////////////////////////////

#endif
