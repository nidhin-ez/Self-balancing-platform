#include <MPU6500-DMP.h>
#include <Wire.h>
#include <ESP32Servo.h>

Servo servo1;
Servo servo2;
MPU6500_DMP imu;

const int servoPin1 = 18;
const int servoPin2 = 19;

float previousPredictedAngle1 = 0;
float previousPredictedAngle2 = 0;
unsigned long prevTime1 = 0;
unsigned long prevTime2 = 0;
unsigned long m = 0;

float tiltAngle1(); // y axis
float tiltAngle2();// xaxis
float gyroRate1();
float gyroRate2();
float filter1(float previousAngle, unsigned long currentTime);
float filter2(float previousAngle, unsigned long currentTime);
void gyroError1();
void  gyroError2();
float gyroE1 = 0;
float gyroE2 = 0;

//PID TERMS

float kP1 = 1.2;
float kI1 = 0;
float kD1 = .03;
float Integral1 = 0;

float kP2 = 1.3;
float kI2 = 0;
float kD2 = .05;
float Integral2 = 0;
float P1 = 0;
float P2 = 0;
float D1 = 0 ;
float D2 = 0;


typedef enum {
  rest,
  start
} state;

state sensor = rest;


//PD control calculation

float PIDcalc1(){
  // the gyro calculates the angular velocity/rate of rotation which would contribute to Kd so we take it instead of differentiating the angle 
  float dt = millis() - prevTime1;

  float e = previousPredictedAngle1;
  float DervE = -1*gyroRate1();
  
  Integral1 += e* (dt/1000); // not using yet ,p-d controller was enough
  
  P1 = kP1*e;
  D1 = kD1*DervE;
  
  float u = kP1*e + kD1*DervE ; 
  float PIDAngle = 90 - (u);
  return PIDAngle;  
}

float PIDcalc2(){
  float dt = millis() - prevTime2;
  float e = previousPredictedAngle2;
  
  Integral2 += e* (dt/1000);  // not using yet ,p-d controller was enough

  float DervE = gyroRate2();

  P2 = kP2*e;
  D2 = kD2*DervE;
  
  float u = kP2*e + kD2*DervE ;
  float PIDAngle = 90 - (u);
  return PIDAngle;  
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);


  if (imu.begin() != INV_SUCCESS) {
    while (1) {
      delay(1000);
      Serial.print("no mpu found!!");
    }
  }
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);  // setting up the accelerometer and the gyro for axis needed

  imu.setAccelFSR(2);      // acceleration limit!!
  imu.setGyroFSR(500);     //rotation speed
  imu.setLPF(20);          // Low Pass Filter for reducing noise from the sensor/ fluctuations
  imu.setSampleRate(300);  // gives the no of times the reading is given out from the sensor per second
}



void runState(unsigned long *currentTime, state *sensorState) {
  switch (*sensorState) {
    case rest:{
        int n = 0;
        while (n < 1) {
          if (imu.dataReady()) {
            imu.update(UPDATE_ACCEL | UPDATE_GYRO);
            previousPredictedAngle1 = tiltAngle1();
            previousPredictedAngle2 = tiltAngle2();
            n++;
          }
        }
        gyroError1();
        gyroError2();
        prevTime1 = millis();
        prevTime2 = millis();
        *sensorState = start;
        *currentTime = millis();
        break;
      }
    case start:{
        if (imu.dataReady()) {
          imu.update(UPDATE_ACCEL | UPDATE_GYRO);

          previousPredictedAngle1 = filter1(previousPredictedAngle1, *currentTime);
          previousPredictedAngle2 = filter2(previousPredictedAngle2, *currentTime);
          servo1.write(PIDcalc1());
          servo2.write(PIDcalc2());
          }
            if(millis() - m >= 20){  // i am using this for serial plotter for monitoring real time graph 
            m = millis();
            Serial.print(previousPredictedAngle2);
            Serial.print(",");
            Serial.print(P2);
            Serial.print(",");
            Serial.print(D2);
            Serial.print(",");
            Serial.print(P2 + D2);
            Serial.print(",");
            Serial.print(kP2,3);
            Serial.print(",");
            Serial.println(kD2,4);
          
          }
          break;
      }
    }
}

float tiltAngle1() { //for y axis
  float accX = imu.calcAccel(imu.ax);
  float accZ = imu.calcAccel(imu.az);
  float angle = atan2(accX, accZ);

  angle = angle * 180 / PI;
  return angle;
}

float tiltAngle2() { // for x axis
  float accY = imu.calcAccel(imu.ay);
  float accZ = imu.calcAccel(imu.az);
  float angle = atan2(accY, accZ);

  angle = angle * 180 / PI;
  return angle;
}

void gyroError1() { // for calculating the error while stationary for y axis
  float gyroV = 0;
  int n = 0;
  while (n < 500) {
    if (imu.dataReady()) {
      imu.update(UPDATE_GYRO);
      gyroV += imu.calcGyro(imu.gy);
      n++;
    }
  }
  gyroE1 = gyroV / 500;
}

void gyroError2() { // for calculating the error while stationary for x axis
  float gyroV = 0;
  int n = 0;
  while (n < 500) {
    if (imu.dataReady()) {
      imu.update(UPDATE_GYRO);
      gyroV += imu.calcGyro(imu.gx);
      n++;
    }
  }
  gyroE2 =  gyroV / 500;
}

float gyroRate1() { // actual gyro reading for y axis
  return (imu.calcGyro(imu.gy) - gyroE1);
}

float gyroRate2() { // actual gyro reading for x axis
  return (imu.calcGyro(imu.gx) - gyroE2);
}


float filter1(float previousAngle, unsigned long currentTime) { //complimentary filter for y axis 
  float dt = (currentTime - prevTime1);
  prevTime1 = currentTime;

  float predictedAngle1 = previousAngle + (-1 * gyroRate1() * (dt / 1000.0));
  
  float CFIL = .980*predictedAngle1 + 0.020*tiltAngle1(); 
  
  return CFIL;
}

float filter2(float previousAngle, unsigned long currentTime) { // complimentary filter for x axis
  float dt = (currentTime - prevTime2);
  prevTime2 = currentTime;

  float predictedAngle2 = previousAngle + (gyroRate2() * (dt / 1000.0));
  
  float CFIL = .980*predictedAngle2 + 0.020*tiltAngle2(); 
  
  return CFIL;
}

void loop() {
  unsigned long currentTime = millis();
  runState(&currentTime, &sensor);
}

