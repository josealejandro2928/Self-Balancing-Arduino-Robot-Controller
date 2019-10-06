#include <Controllers.h>
#include <ComplementaryFilter.h>
#include "Wire.h"
#include "MPU6050.h"
#include "I2Cdev.h"
#include "math.h"
////////////////////Bluetooth Protocol////
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
unsigned char bufferData[4] = {0, 0, 0, 0};
//////////////Robot Constant////////////
#define MIN_ABS_SPEED 58              //// Minima señal de PWM a la que los motores se mueven : punto muerto para los motores
#define SAMPLE_TIME_VELOCITY 20000    //microsegundos - 20 ms PID de velocidad
#define SAMPLE_TIME_INCLINATION 10000 //microsegundos - 10 ms PID de inclinacion
#define REVOLUTION_STEPS 1920.0
#define WhEEL_RADIUS 0.048
#define WhEEL_DISTANCE 0.245
#define ROBOT_HEIGHT 0.30
#define PULSES_FACTOR ((2.0f * PI) / REVOLUTION_STEPS)
////////////ENCODER VARIABLES///////////////
#define ENCODER_M1_A 3
#define ENCODER_M1_B 18
#define ENCODER_M2_A 2
#define ENCODER_M2_B 19
volatile int stateEA_M1 = 1; //RISING
volatile int stateEB_M1 = 1; //RISING
volatile int stateEA_M2 = 1; //RISING
volatile int stateEB_M2 = 1; //RISING
//////////////////////////ROBOT ESTATE/////////////////////////
float inclination = 0.0;           ///degrees
float velocity = 0.0;              ////m/s
float angular_velocity = 0.0;      //// rad/s
float angular_velocity_gyro = 0.0; ///rad/s
float angular_velocity_encoders = 0.0; ///rad/s
float diff_angular_rate = 0.0;////rad/s
float robotX = 0.0;                ///m
float robotY = 0.0;                ///m
float robotTheta = 0.0;            ///rad
float speed_M1 = 0.0;              ////rad/s
float speed_M2 = 0.0;              ////rad/s
float L_M1 = 0.0;                  ////displacement of wheel M1
float L_M2 = 0.0;                  ////displacement of wheel M2
int GO2GoalMode = 0; 
unsigned int  recargaTimer = 3035;////Timer1 Recharger to 25ms
///////////////////////MPU Global Variables////////////////////
MPU6050 sensor;
int ax, ay, az;
int gx, gy, gz;
ComplementaryFilter CF(0.98, 0.02, 0.0);
/////////////////////////////////////////////////////////
float inclination_ax = 0.0;
float roll_gy = 0.0;
float last_roll_gy = 0.0;
float gyro_angle = 0.0;
float comp_angle = 0.0;
float linear_accel = 0.0;
////////////////////////////PID Controllers//////////////////////////

///////////Balancing Controller/////////////////////////////////////
float sp_inclination = 0.0;
float angle0 = -1.25;
float PWM_output = 0.0;
float Kc_i = 23.75;
float Ki_i = 20.5;
float Kd_i = 1.75;
///////////Velocity Controller///////////////////////////////////////
float sp_velocity = 0.0;
float Kc_v = 9.75;
float Ki_v = 5.25;
float Kd_v = 0.155;
float max_angle_output = 8.5;
////////////////////// Steering Controller ///////////////////////
float sp_angular_velocity = 0.0;
float Kc_w = 20.5;
float Ki_w = 12.5;
float Kd_w = 0.15;
float PWM_W_controller = 0.0; ///Salida en PWM////
////////////////////////Point Tracker Controller///////////
float Kc_pos = 1.0;
float Ki_pos = 2.0;
float Kd_pos = 0.005;
float sum_pos_error = 0.0;
float last_pos_error = 0.0;
float MAX_VEL_PT = 0.45;
float MAX_ANGULAR_VEL_PT = 3.0;
float TRESHOLD_PT = 0.02;
float Kc_orient = 1.75;
float sp_posX = 0.0;
float sp_posY = 0.0;
char modePointTracker =' ';

///////Kalman FIlter EStimator for Velocity//////////////
////////MotorM1/////
float velocity_KF = 0.0;
float speed_M1_KF = 0.0;
float Q_v_M1 = 1.0;
float R_v_M1 = 0.05;
float P_KF_M1 = 0.0;
float G_KF_M1 = 0.0;
float alphaM1 = 250.0;
float ganmaM1 = 1.0;
////////MotorM2//////
float speed_M2_KF = 0.0;
float Q_v_M2 = 1.0;
float R_v_M2 = 0.05;
float P_KF_M2 = 0.0;
float G_KF_M2 = 0.0;  
float alphaM2 = 250.0;
float ganmaM2 = 1.0;
//////////////////MOTORS Global Variables//////////////////////
unsigned long prev_time_inclination = 0; //////guardan el tiempo anterior para el calculo del tiempo en que efectuan control
unsigned long prev_time_velocity = 0;    //////guardan el tiempo anterior para el calculo del tiempo en que efectuan control
volatile long ticks_M1 = 0;
volatile long ticks_M2 = 0;
long last_ticks_M1 = 0;
long last_ticks_M2 = 0;
/////////////Motor 1 Pines///////////
int ENA = 6;
int IN1 = 9;
int IN2 = 8;
////////////Motor 2 Pines ////////////
int ENB = 5;
int IN3 = 10;
int IN4 = 11;
float motorSpeedFactorM1 = 1.0;
float motorSpeedFactorM2 = 1.0;
volatile int senseM1 = 0;
volatile int senseM2 = 0;
int deadZone = 0;
/////////////////////////////////////////////////////////////////
///////////Batery Variables/////////
int pinA0 = A0;
volatile float charge = 0.0;

void setup()
{
    Serial.begin(115200);
    Serial3.begin(115200);
    delay(350);
    MPU_Inicialization();
    L298N_Init();
    initInterruptsEncoders();
    pinMode(A0,INPUT);
    prev_time_inclination = micros();
    prev_time_velocity = micros();
    interrupts();
    initSamplingBattery();
}

void loop()
{

    unsigned long now_time = micros();
    unsigned long dt = (now_time - prev_time_inclination);

    /////////////PID de Inclinacion////////////////
    if (dt >= SAMPLE_TIME_INCLINATION)
    {
        MPU_measurement();
        float s_time = (float)(dt / 1000000.0);
        getMeassurement(s_time);
        inclination = comp_angle;
        PWM_output = -1.0 * inclinationPID(sp_inclination, inclination, 255.0, s_time, Kc_i, Ki_i, Kd_i);
        prev_time_inclination = micros();
        /////////////////IMPRIMIENDO RESULTADOS////////////////
    }

    now_time = micros();
    dt = (now_time - prev_time_velocity);

    ///////PID de Velocidad Lineal y Angular/////////////////
    if (dt >= SAMPLE_TIME_VELOCITY)
    {    
        float s_time = (float)(dt / 1000000.0);
        getState(s_time);
        sp_inclination = velocityPID(sp_velocity, velocity_KF, max_angle_output, s_time, Kc_v, Ki_v, Kd_v) + angle0;
        PWM_W_controller = angularVelocityPID(sp_angular_velocity, angular_velocity, 50.0, s_time, Kc_w, Ki_w, Kd_w);
        if(GO2GoalMode){
            PointTracker(s_time);
        }
        prev_time_velocity = micros();
      ///////////Printiando Resultados para observar Comportamiento//////////////////
//      Serial.print(speed_M1);
//      Serial.print('\t');
//      Serial.println(speed_M1_KF);  
    }
    
    BluetoothSerial();
    //L298N_move(250,0);
    L298N_move((PWM_output + PWM_W_controller), (PWM_output - PWM_W_controller));
 
}

//////////GETTING THE STATE///////////////////////////////////
void getState(float dt)
{
    
    long delta_ticks_M1 = (ticks_M1 - last_ticks_M1);
    long delta_ticks_M2 = -1 * (ticks_M2 - last_ticks_M2);

    //////Velocidades de cada rueda del robot/////////////
    speed_M1 = ((float)(delta_ticks_M1)*PULSES_FACTOR) / dt;
    speed_M2 = ((float)(delta_ticks_M2)*PULSES_FACTOR) / dt;
    ///////////////////Update Count Ticks/////////////////
    last_ticks_M1 = ticks_M1;
    last_ticks_M2 = ticks_M2;
    
    ////////Estimacion con Filtro de Kalman///////////////
    KF_DC_Speed(speed_M1,speed_M2,dt);
    ///////////////////Velocidad lineal del Robot/////////
    velocity = (WhEEL_RADIUS * (speed_M1 + speed_M2)) / 2.0;
    velocity_KF = (WhEEL_RADIUS * (speed_M1_KF + speed_M2_KF)) / 2.0;
    
    ///////////////////// Velocidad angular del robot ///////////////////////
    angular_velocity_gyro = -1.0 * radians((float)(sensor.getRotationZ()) / 131.072);
    angular_velocity_encoders = ((speed_M1 - speed_M2) * (WhEEL_RADIUS / WhEEL_DISTANCE));
    angular_velocity = 0.75 * angular_velocity_gyro + 0.25 * angular_velocity_encoders;
    diff_angular_rate = angular_velocity_gyro - angular_velocity_encoders;
    ///////////////////POSICION en X,Y,orientación//////////////////////////
    robotX = robotX + velocity*dt*cos(robotTheta + (angular_velocity*dt)/2.0);
    robotY = robotY + velocity*dt*sin(robotTheta + (angular_velocity*dt)/2.0);
    // robotTheta = angle2PI( angle2PI(robotTheta) + angle2PI(angular_velocity_encoders*dt) );
    if( abs(diff_angular_rate) >= 0.25 ){
        robotTheta = angle2PI( angle2PI(robotTheta) + angle2PI(angular_velocity_gyro*dt) );
    }
    else{
        robotTheta = angle2PI( angle2PI(robotTheta) + angle2PI(angular_velocity_encoders*dt) );
    }
    
}

///////INTERRUPTS ENCODERS API//////////////////////////////
void initInterruptsEncoders()
{
    pinMode(ENCODER_M1_A, INPUT);
    pinMode(ENCODER_M1_B, INPUT);
    pinMode(ENCODER_M2_A, INPUT);
    pinMode(ENCODER_M2_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_M1_A), count_ticks_M1A, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_M1_B), count_ticks_M1B, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_M2_A), count_ticks_M2A, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_M2_B), count_ticks_M2B, RISING);
    ticks_M1 = 0;
    ticks_M2 = 0;
}
/////////////////////////////////////////////////////////////
////////////////////////////////MPU SENSOR API////////////////////////////////
void MPU_Inicialization()
{
    Wire.begin();
    TWSR = (TWSR & 0xFC);
    TWBR = 12;
    delay(20);
    sensor.initialize();
    delay(20);
    settingOffset(-1818, 344, 1257, -252, 140, 21);
    delay(50);
}

void MPU_measurement()
{
    sensor.getAcceleration(&ax, &ay, &az);
    //sensor.getRotation(&gx, &gy, &gz);
    gy = sensor.getRotationY();
    roll_gy = (float)(gy) / 131.072;
    inclination_ax = degrees(atan(1.0 * ax / sqrt(pow(ay, 2.0) + pow(az, 2.0))));
}

void getMeassurement(float dt)
{
    comp_angle = CF.get_measurement(inclination_ax, roll_gy, dt);
    linear_accel = (ax - sin(radians(comp_angle)) * 16384.0) * (0.000598144);
//    linear_accel = cos(radians(comp_angle))*(linear_accel - ( radians(roll_gy) - radians(last_roll_gy) )*(ROBOT_HEIGHT/dt) );
    last_roll_gy = roll_gy;
}

void settingOffset(int ax, int ay, int az, int gx, int gy, int gz)
{
    sensor.setXAccelOffset(ax);
    sensor.setYAccelOffset(ay);
    sensor.setZAccelOffset(az);
    sensor.setXGyroOffset(gx);
    sensor.setYGyroOffset(gy);
    sensor.setZGyroOffset(gz);
}
///////////////////////////////////////////////////////////////////////
///////////////////INTERRUPTS SUBRUTINES///////////////////////////////
void count_ticks_M1A()
{ ///////////Motor 1 encoder A//////////

    if (stateEA_M1)
    {
        ticks_M1 += digitalRead(ENCODER_M1_B) ? -1 : 1;
        attachInterrupt(digitalPinToInterrupt(ENCODER_M1_A), count_ticks_M1A, FALLING);
        stateEA_M1 = 0;
    }
    else
    {
        ticks_M1 += digitalRead(ENCODER_M1_B) ? 1 : -1;
        attachInterrupt(digitalPinToInterrupt(ENCODER_M1_A), count_ticks_M1A, RISING);
        stateEA_M1 = 1;
    }
}
void count_ticks_M1B()
{ ///////////Motor 1 encoder B//////////
    if (stateEB_M1)
    {
        ticks_M1 += digitalRead(ENCODER_M1_A) ? 1 : -1;
        attachInterrupt(digitalPinToInterrupt(ENCODER_M1_B), count_ticks_M1B, FALLING);
        stateEB_M1 = 0;
    }
    else
    {
        ticks_M1 += digitalRead(ENCODER_M1_A) ? -1 : 1;
        attachInterrupt(digitalPinToInterrupt(ENCODER_M1_B), count_ticks_M1B, RISING);
        stateEB_M1 = 1;
    }
}
void count_ticks_M2A()
{ ///////////Motor 2 encoder A//////////
    if (stateEA_M2)
    {
        ticks_M2 += digitalRead(ENCODER_M2_B) ? -1 : 1;
        attachInterrupt(digitalPinToInterrupt(ENCODER_M2_A), count_ticks_M2A, FALLING);
        stateEA_M2 = 0;
    }
    else
    {
        ticks_M2 += digitalRead(ENCODER_M2_B) ? 1 : -1;
        attachInterrupt(digitalPinToInterrupt(ENCODER_M2_A), count_ticks_M2A, RISING);
        stateEA_M2 = 1;
    }
}
void count_ticks_M2B()
{ ///////////Motor 2 encoderB//////////
    if (stateEB_M2)
    {
        ticks_M2 += digitalRead(ENCODER_M2_A) ? 1 : -1;
        attachInterrupt(digitalPinToInterrupt(ENCODER_M2_B), count_ticks_M2B, FALLING);
        stateEB_M2 = 0;
    }
    else
    {
        ticks_M2 += digitalRead(ENCODER_M2_A) ? -1 : 1;
        attachInterrupt(digitalPinToInterrupt(ENCODER_M2_B), count_ticks_M2B, RISING);
        stateEB_M2 = 1;
    }
}
///////////////////////////////////////////////////////////////////////
////////////LN298N///////////////////////////////////////////////////
void L298N_Init()
{
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

void L298N_move(int speedM1, int speedM2)
{
    if (speedM2 < -deadZone)
    {
        speedM2 = constrain(abs(speedM2), MIN_ABS_SPEED, 255);
        senseM2 = -1;
    }
    else if (speedM2 > deadZone)
    {
        speedM2 = constrain(abs(speedM2), MIN_ABS_SPEED, 255);
        senseM2 = 1;
    }
    else
    {
        senseM2 = 0;
        speedM2 = 0;
    }

    if (speedM1 < -deadZone)
    {
        speedM1 = constrain(abs(speedM1), MIN_ABS_SPEED, 255);
        senseM1 = -1;
    }
    else if (speedM1 > deadZone)
    {
        speedM1 = constrain(abs(speedM1), MIN_ABS_SPEED, 255);
        senseM1 = 1;
    }
    else
    {
        speedM1 = 0;
        senseM1 = 0;
    }

    digitalWrite(IN3, senseM2 > 0 ? HIGH : LOW);
    digitalWrite(IN4, senseM2 > 0 ? LOW : HIGH);
    digitalWrite(IN1, senseM1 > 0 ? HIGH : LOW);
    digitalWrite(IN2, senseM1 > 0 ? LOW : HIGH);
    analogWrite(ENA, (int)abs(speedM1 * motorSpeedFactorM1));
    analogWrite(ENB, (int)abs(speedM2 * motorSpeedFactorM2));
}
///////////////Battery Status//////////////
void getBattery(){
  float x = 1.0206*analogRead(pinA0) - 886.9202;
  charge = 0.75*charge + 0.25*x;
  charge = constrain(charge,0.0,100.0);
  }

//////TiMer Configuration for Sampling the battery ////
void initSamplingBattery()
{
  TCCR1A = 0x00; 
  TCCR1B = 0x04;  
  TCNT1H = ((recargaTimer>>8)&0x00FF); 
  TCNT1L = ((recargaTimer)&0x00FF); 
  TIMSK1 = 0x01; 
  TIFR1 = 0x00;  
}

//////////////////////////////////////////////////
//////// Bluetooth Protocol Interface////////
void  BluetoothSerial()
{
  if (Serial3.available() > 0)
  {
    char command = Serial3.read();
    //Serial.println(command);
    
    if (command == COMMAND_SETPOINT_SPEEDS)
    {
      while (Serial3.available() < 4);
      Serial3.readBytes(bufferData, 4);
      sp_velocity = bytesToFloat(bufferData);
      while (Serial3.available() < 4);
      Serial3.readBytes(bufferData, 4);
      sp_angular_velocity = bytesToFloat(bufferData);
      GO2GoalMode = 0;
      sum_pos_error = 0.0;
    }
    else if (command == POINT_TRACKER_MODE)
    {
      while (Serial3.available() < 4);
      Serial3.readBytes(bufferData, 4);
      sp_posX = bytesToFloat(bufferData);
      while (Serial3.available() < 4);
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
      while (Serial3.available() < 4);
      Serial3.readBytes(bufferData, 4);
      Kc_i = bytesToFloat(bufferData);
      while (Serial3.available() < 4);
      Serial3.readBytes(bufferData, 4);
      Ki_i = bytesToFloat(bufferData);
      while (Serial3.available() < 4);
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
      while (Serial3.available() < 4);
      Serial3.readBytes(bufferData, 4);
      Kc_v = bytesToFloat(bufferData);
      while (Serial3.available() < 4);
      Serial3.readBytes(bufferData, 4);
      Ki_v = bytesToFloat(bufferData);
      while (Serial3.available() < 4);
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
      while (Serial3.available() < 4);
      Serial3.readBytes(bufferData, 4);
      Kc_w = bytesToFloat(bufferData);
      while (Serial3.available() < 4);
      Serial3.readBytes(bufferData, 4);
      Ki_w = bytesToFloat(bufferData);
      while (Serial3.available() < 4);
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
      sum_pos_error = 0.0;
      sp_velocity = 0.0;
      sp_angular_velocity = 0.0;
    }

  }

}

float bytesToFloat(unsigned char data[4])
{
  float output;
  *((unsigned char *)(&output) + 3) = data[3];
  *((unsigned char *)(&output) + 2) = data[2];
  *((unsigned char *)(&output) + 1) = data[1];
  *((unsigned char *)(&output) + 0) = data[0];
  return output;
}

/////////SUBRRUTINE To Timer1 INTERRUPT//////////////
ISR(TIMER1_OVF_vect){
  getBattery();
}

float angle2PI(float angle){
    if(angle > 0.0){
        angle  = ((angle >= (2*PI)))?(angle-(2*PI)):angle;
    }
    else if (angle < 0.0){
        angle = (angle >= -(2*PI))?(angle + (2*PI)): -1.0*(angle+(2*PI));
    }
    return angle;
}

void PointTracker(float dt){
    
    float theta_forward = robotTheta;
    float theta_backward = robotTheta;
    float posError = 0.0;
    float orientError = 0.0;
    float u_pos = 0.0;
    float u_orient = 0.0;
    

    if( theta_forward > PI && theta_forward < (2*PI) ){
        theta_forward = theta_forward - (2*PI);
    }
    theta_backward = theta_forward - PI;
    float dist_to_goal = sqrt( pow((sp_posX - robotX),2.0) + pow((sp_posY - robotY),2.0) );
    float theta_goal   = atan2(sp_posY - robotY , sp_posX - robotX);

    //////////Cambio de modos///////////
    float backward_boundery = angle2PI(theta_backward) - angle2PI(theta_goal);
    if( (abs(backward_boundery)<= 1.57) && dist_to_goal <= 1.25){
        modePointTracker = 'B';
    }
    else{
        modePointTracker = 'F'; 
    }

    //////////////////////////////////////////// 


    if(modePointTracker == 'F'){
        //////// Orientation Controller/////
        float dist1 = angle2PI(theta_goal) - angle2PI(theta_forward);
        float dist2 = theta_goal - theta_forward;
        if(abs(dist1) < abs(dist2)){
            orientError = dist1;
        }
        else if(abs(dist1) > abs(dist2)){
            orientError = dist2;
        }
        else{
            orientError = dist2;
        }
        
        /////////Orientacion Control//////////
        u_orient = constrain(Kc_orient*orientError, -1.0*MAX_ANGULAR_VEL_PT , MAX_ANGULAR_VEL_PT);

        //////// Position Controller/////
        posError = dist_to_goal;
        sum_pos_error = Ki_pos * ( sum_pos_error + posError ) * dt;
        sum_pos_error = constrain(sum_pos_error , -1.0 * MAX_VEL_PT , MAX_VEL_PT);
        float I_term = sum_pos_error;
        float D_term =  Kd_pos * (posError - last_pos_error)/dt;
        u_pos = constrain(Kc_pos*posError + I_term + D_term , -1.0 * MAX_VEL_PT , MAX_VEL_PT);
        last_pos_error = posError;
        
    }
    else{
        float dist1 = angle2PI(theta_goal) - angle2PI(theta_backward); 
        float dist2 = theta_goal - theta_backward;
        if(abs(dist1) < abs(dist2)){
            orientError = dist1;
        }
        else if(abs(dist1) > abs(dist2)){
            orientError = dist2;
        }
        else{
            orientError = dist1;
        }
        /////////Orientacion Control//////////
        u_orient = constrain(Kc_orient*orientError, -1.0*MAX_ANGULAR_VEL_PT , MAX_ANGULAR_VEL_PT);
        
        //////// Position Controller/////
        posError = dist_to_goal;
        sum_pos_error = Ki_pos * ( sum_pos_error + posError ) * dt;
        sum_pos_error = constrain( sum_pos_error , -1.0 * (MAX_VEL_PT - 0.05) , (MAX_VEL_PT- 0.05) );
        float I_term = sum_pos_error;
        float D_term =  Kd_pos * (posError - last_pos_error)/dt;
        u_pos = -1.0 * constrain( Kc_pos*posError + I_term + D_term , -1.0 * (MAX_VEL_PT - 0.05) , (MAX_VEL_PT - 0.05) );
        last_pos_error = posError;
    }

    if( abs(posError) <= TRESHOLD_PT ){
        u_pos = 0.0;
        u_orient = 0.0;
    }
    
    sp_velocity = u_pos;
    sp_angular_velocity = u_orient;
}

/////////Kalman Filter estimator for DC motor //////////////////
void KF_DC_Speed(float speedM1_E , float speedM2_E , float dt){
    // ----------------------------------------
    float taoM1 = speedM1_E - speed_M1_KF;
    float taoM2 = speedM2_E - speed_M2_KF;
    // ----------------------------------------
    Q_v_M1 = ((alphaM1*alphaM1)*(taoM1*taoM1)*dt*dt)/(1+ganmaM1*speedM1_E*speedM1_E);
    Q_v_M2 = ((alphaM2*alphaM2)*(taoM2*taoM2)*dt*dt)/(1+ganmaM2*speedM2_E*speedM2_E);
    // ----------------------------------------
    float priori_SpeedM1 = speed_M1_KF;
    float priori_P_KF_M1 = P_KF_M1 + Q_v_M1;
    float priori_SpeedM2 = speed_M2_KF;
    float priori_P_KF_M2 = P_KF_M2 + Q_v_M2;
    // ----------------------------------------
    G_KF_M1 = priori_P_KF_M1/(priori_P_KF_M1 + R_v_M1);
    G_KF_M2 = priori_P_KF_M2/(priori_P_KF_M2 + R_v_M2);
    // ----------------------------------------
    speed_M1_KF = priori_SpeedM1 + G_KF_M1*(speedM1_E - priori_SpeedM1);
    P_KF_M1 = (1-G_KF_M1)*priori_P_KF_M1;
    speed_M2_KF = priori_SpeedM2 + G_KF_M2*(speedM2_E - priori_SpeedM2);
    P_KF_M2 = (1-G_KF_M2)*priori_P_KF_M2;

}
