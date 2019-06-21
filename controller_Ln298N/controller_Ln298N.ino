#include <PID.h>
#include <Wire.h>
#include <KalmanFilterPro.h>
#include <ComplementaryFilter.h>
#include "MPU6050.h"
#include "I2Cdev.h"
#include "math.h"
#define REVOLUTION_STEPS 90
/////////////////GLOBAL VRIABLES///////////////
////////////////SAMPLE VARIABLES//////////////////////
char buffer[4];
volatile unsigned int recarga = 0;
volatile unsigned int FS = 50; ///Hz////
volatile double SAMPLE_TIME = (double)(1.0 / ((double)(FS)));
/////////////DC MOTORS//////////////////////
volatile int minAbsMotorPWM = 50;
volatile double minUsignalTreshold = 5.0;
int M1_in1 = 8;
int M1_in2 = 9;
int M1_pwm = 13; //////////////pin 13 in Atmega/// 5 pin in Uno
int M2_in1 = 10;
int M2_in2 = 11;
int M2_pwm = 4; /////////////pin 4 in Atmega/// 6  pin in Uno
volatile long count_pulses_M1 = 0;
volatile long count_pulses_M2 = 0;
volatile int senseM1 = 0;
volatile int senseM2 = 0;
int encoderM1A_interrupt = 0; ///pin Digital 2 Atmega INT4
int encoderM2A_interrupt = 1; ///pin Digital 3 Atmega INT5
volatile float setpoint_v1 = 0.0;
volatile float setpoint_v2 = 0.0;
volatile float setpoint_theta = 0.5;
volatile float pulses_factorM1 = 0.0;
volatile float pulses_factorM2 = 0.0;
volatile long long last_pulses_M1 = 0.0;
volatile long long last_pulses_M2 = 0.0;
volatile float speedM1 = 0.0;
volatile float speedM2 = 0.0;
///////////MPU INTERFACE///////////////////////////////
///////////////////////////////////////////////////
MPU6050 sensor;
int ax, ay, az, gx, gy, gz = 0;
ComplementaryFilter CF(0.975, 0.025, 0.0);
KalmanFilterPro KF2(0.0);
volatile double inclination_ax = 0.0;
volatile double roll_gy = 0.0;
volatile double gyro_angle = 0.0;
volatile double comp_angle = 0.0;
volatile double Kf_angle = 0.0;
///////////////////////////////////////////////
////////////PIDs/////////////////////////////////////
/*PID M1_Controller = PID(10, 1.5, 3.25, 0.0, 255);
PID M2_Controller = PID(10, 1.5, 3.25, 0.0, 255);*/
PID theta_Controller = PID(40, 20.0, 1.25, 0.0, 255);
/////////////////////////////////////////////////////
void setup()
{
  recarga = 65536 - (16 * 1000000) / (FS * 64);
  Serial.begin(115200);
  delay(1);
  initMotors();
  /*attachInterrupt(encoderM1A_interrupt,catch_M1A_pulses,FALLING);
  attachInterrupt(encoderM2A_interrupt,catch_M2A_pulses,FALLING);*/
  Wire.begin();
  MPU_Init();
  delay(1);
  interrupts();
  initSampling();
}

void loop()
{
  MPU_measurement();
}
////////////////////////////////////MPU API/////////////////////////////////
////////////////////////////////////////////////////////////////////////////
void MPU_Init()
{
  sensor.initialize(); //Iniciando el sensor/////
  if (sensor.testConnection())
    Serial.println("Sensor iniciado correctamente");
  else
    Serial.println("Error al iniciar el sensor");
  settingOffset(-1702, 355, 1352, -192, 274, 34);
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

void MPU_measurement()
{
  ax = sensor.getAccelerationX();
  ay = sensor.getAccelerationY();
  ay = sensor.getAccelerationZ();
  gx = sensor.getRotationX();
  gy = sensor.getRotationY();
  gz = sensor.getRotationZ();
  roll_gy = (double)((double)(gy) / 131.072);
  inclination_ax = degrees(atan(ax / sqrt(pow(ay, 2.0) + pow(az, 2.0))));
}

void getMeassurement(double dt)
{
  /////////////////////////////////////////////////
  Kf_angle = KF2.get_state(inclination_ax, roll_gy, dt);
  comp_angle = CF.get_measurement(inclination_ax, roll_gy, dt);
  /////////////////////////////////////////////////
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////// MOTORS FUNCTIONS ////////////////////////////
void InitPWM()
{
  TCCR0A = 0xA3; // fast pwm mode, non inverted for both
  TCCR0B = 0x01; // clkio => timer frequency
  TCNT0 = 0x00;  // reset count
  OCR0A = 0;     // 0 pwm2
  OCR0B = 0;     // 0 pwm1
}

void initSampling()
{
  TCCR1A = 0x00;
  TCCR1B = 0x03;
  TCNT1H = ((recarga >> 8) & 0x00FF);
  TCNT1L = ((recarga)&0x00FF);
  TIMSK1 = 0x01;
  TIFR1 = 0x00;
}

void initMotors()
{
  pulses_factorM1 = 2.0f * PI / REVOLUTION_STEPS / SAMPLE_TIME;
  pulses_factorM2 = 2.0f * PI / REVOLUTION_STEPS / SAMPLE_TIME;
  pinMode(M1_in1, OUTPUT);
  pinMode(M1_in2, OUTPUT);
  pinMode(M1_pwm, OUTPUT);
  pinMode(M2_in1, OUTPUT);
  pinMode(M2_in2, OUTPUT);
  pinMode(M2_pwm, OUTPUT);
  InitPWM();
  sendspeed_M1(0);
  sendspeed_M2(0);
  setpoint_v1 = 0.0;
  setpoint_v2 = 0.0;
}

void sendspeed_M1(double u)
{
  if (u < -1.0 * minUsignalTreshold)
  {
    u = min(u, -1.0 * minAbsMotorPWM);
    digitalWrite(M1_in1, 1);
    digitalWrite(M1_in2, 0);
    senseM1 = -1;
  }
  else if (u > minUsignalTreshold)
  {
    u = max(u, minAbsMotorPWM);
    digitalWrite(M1_in1, 0);
    digitalWrite(M1_in2, 1);
    senseM1 = 1;
  }
  else
  {
    digitalWrite(M1_in1, 0);
    digitalWrite(M1_in2, 0);
    senseM1 = 0;
  }

  OCR0B = abs((int)(u));
}
void sendspeed_M2(double u)
{
  if (u < -1.0 * minUsignalTreshold)
  {
    u = min(u, -1.0 * minAbsMotorPWM);
    digitalWrite(M2_in1, 0);
    digitalWrite(M2_in2, 1);
    senseM2 = -1;
  }
  else if (u > minUsignalTreshold)
  {
    u = max(u, minAbsMotorPWM);
    digitalWrite(M2_in1, 1);
    digitalWrite(M2_in2, 0);
    senseM2 = 1;
  }
  else
  {
    digitalWrite(M2_in1, 0);
    digitalWrite(M2_in2, 0);
    senseM2 = 0;
  }
  OCR0A = abs((int)(u));
}
///////////////////////INTERRUPTS SUBRRUTINES/////////////////////
//////////////////////////////////////////////////////////////////
void catch_M1A_pulses()
{
  if (senseM1 == 1)
    count_pulses_M1++;
  if (senseM1 == -1)
    count_pulses_M1--;
}
void catch_M2A_pulses()
{
  if (senseM2 == 1)
    count_pulses_M2++;
  if (senseM2 == -1)
    count_pulses_M2--;
}
/////////////////////////////////////SAMPLING FUNCTION/////////////////////////////////
ISR(TIMER1_OVF_vect)
{
  TCNT1H = ((recarga >> 8) & 0x00FF);
  TCNT1L = ((recarga)&0x00FF);
  /////////speedM1///////////////////////////////
  /*long temp_pulses1 = count_pulses_M1;
  long delta_pulses1 = temp_pulses1 - last_pulses_M1;
  last_pulses_M1 = temp_pulses1;
  ////////////speedM2///////////////////////////////
  long temp_pulses2 = count_pulses_M2;
  long delta_pulses2 = temp_pulses2 - last_pulses_M2;
  last_pulses_M2 = temp_pulses2;
  speedM1 = (float)(delta_pulses1) *  pulses_factorM1;
  speedM2 = (float)(delta_pulses2) * pulses_factorM2;
  /////////////////////////////////////////////////
  double u1 = M1_Controller.velocity_compute(setpoint_v1,speedM1);
  double u2 = M2_Controller.positional_compute(setpoint_v2,speedM2);*/
  getMeassurement(SAMPLE_TIME);
  double u = theta_Controller.classic_compute(setpoint_theta, comp_angle, SAMPLE_TIME);
  Serial.println(comp_angle);
  /*Serial.print('\t');
  Serial.println(u);*/
  sendspeed_M1(u);
  sendspeed_M2(u);
}
