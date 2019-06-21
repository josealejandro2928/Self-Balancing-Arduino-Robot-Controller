#include <KalmanFilterPro.h>
#include <KalmanFilter1D.h>
#include <ComplementaryFilter.h>
#include "Wire.h"
#include "MPU6050.h"
#include "I2Cdev.h"
#include "math.h"

////////////////SAMPLE VARIABLES//////////////////////
volatile unsigned int recarga = 0;
volatile unsigned int FS = 60;///Hz////
volatile double SAMPLE_TIME = (double)(1.0/((double)(FS)));
///////////////////////////////////////////////////////
///////////MPU INTERFACE///////////////////////////////
MPU6050 sensor;
volatile int ax, ay, az;
volatile int gx, gy, gz;
ComplementaryFilter CF(0.975,0.025,0.0);
KalmanFilter1D KF1(0.0);
KalmanFilterPro KF2(0.0);
/////////////////////////////////////////////////////////
volatile double inclination_ax = 0.0;
volatile double roll_gy = 0.0;
volatile double gyro_angle = 0.0;
volatile double comp_angle = 0.0;
volatile double Kf_angle = 0.0;
volatile double median_both_filters = 0.0;
volatile int counter = 0;
//////////////////////////////////////////////////////
void setup() {
  recarga = 65536 - (16*1000000)/(FS*64);
  Serial.begin(115200);    //Iniciando puerto serial
  delay(10);
  pinMode(21,INPUT);
  pinMode(20,INPUT);
  Wire.begin();
  delay(10);           //Iniciando I2C
  sensor.initialize();    //Iniciando el sensor
  delay(10);

  if (sensor.testConnection())
      Serial.println("Sensor iniciado correctamente");
  else 
      Serial.println("Error al iniciar el sensor");
  
  settingOffset(-1818,344,1257,-252,140,21);
  delay(100);
  //interrupts();
  initSampling();

}

void loop() {
  MPU_measurement();
  if(counter == 1){
      //Serial.println("//////////////");
      //Serial.print("inclination_ax: ");
    
     // Serial.print(inclination_ax);
      //Serial.print('\t');
      Serial.print(comp_angle);
      Serial.print('\t');
      //Serial.print("kalman_angle: ");
      Serial.println(Kf_angle);
      //Serial.print("median_both_filters: ");
      //Serial.println(median_both_filters);
      //Serial.println("//////////////");
      counter = 0;
  }
  
}

void settingOffset(int ax,int ay,int az,int gx,int gy,int gz){
  sensor.setXAccelOffset(ax);
  sensor.setYAccelOffset(ay);
  sensor.setZAccelOffset(az);
  sensor.setXGyroOffset(gx);
  sensor.setYGyroOffset(gy);
  sensor.setZGyroOffset(gz);
}
void MPU_measurement(){
  ax = sensor.getAccelerationX();
  ay = sensor.getAccelerationY();
  ay = sensor.getAccelerationZ();
  gx = sensor.getRotationX();
  gy = sensor.getRotationY();
  gz = sensor.getRotationZ();
  roll_gy =-1.0*(double)((double)(gy)/131.072);
  inclination_ax = degrees(atan(-1.0*ax/sqrt(pow(ay,2.0) + pow(az,2.0))));
}

void getMeassurement(){
  //////////////////////////////////////////////
  double dt = SAMPLE_TIME;
  Kf_angle = KF2.get_state(inclination_ax,roll_gy,dt);
  comp_angle =  CF.get_measurement( inclination_ax , roll_gy, dt);
  /////////////////////////////////////////////////
}

void initSampling()
{
  TCCR1A = 0x00; 
  TCCR1B = 0x03;  
  TCNT1H = ((recarga>>8)&0x00FF); 
  TCNT1L = ((recarga)&0x00FF); 
  TIMSK1 = 0x01; 
  TIFR1 = 0x00; 
}

ISR(TIMER1_OVF_vect){
  TCNT1H = ((recarga>>8)&0x00FF); 
  TCNT1L = ((recarga)&0x00FF);
  getMeassurement();
  median_both_filters = 0.5*Kf_angle + 0.5*comp_angle;
  counter++;
}
