///////////////////////////////////////////
#include "LMotorController.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <PinChangeInt.h>
///////////////////////////////////////////
///mpu///
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 #include "Wire.h"
#endif
MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[1]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
 mpuInterrupt = true;
}
/////////////////////////


//MOTOR CONTROLLER
double motorSpeedFactorLeft = 0.5;
double motorSpeedFactorRight = 0.5;
int ENA = 6;
int IN1 = 7;
int IN2 = 8;
int IN3 = 4;
int IN4 = 5;
int ENB = 3;
#define MIN_ABS_SPEED 18
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);
////////////////

///encoder///
volatile int en1=0,en2=0;
int motorAngularPosition=0;
float motorAngularSpeed=0;
const float gearRatio=51.45;
const float countToDegrees = (360 / (12.0 * gearRatio));
int disF=0;

int GradosP=360/150;

#define s1 11
#define s2 12

#define sensor A2
/////////////////

//lqr///
float x1,x2,x3,x4;
float x1_ant=0;
float x3_ant=0;
float k[4]={-0.5,-0.9,33,150};//-0.8,-0.5,26,100
int SetPoint=0;
float input;
double dt=0,dt2=0.0001;
volatile double U;
////////////////////



void setup() {
  Serial.begin(2000000);
 ////////MPU//////// 
 // join I2C bus (I2Cdev library doesn't do this automatically)
 #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
 #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
 Fastwire::setup(400, true);
 #endif

 mpu.initialize();

 devStatus = mpu.dmpInitialize();

 // supply your own gyro offsets here, scaled for min sensitivity
 mpu.setXGyroOffset(63);
 mpu.setYGyroOffset(40);
 mpu.setZGyroOffset(22);
 mpu.setZAccelOffset(976); // 1688 factory default for my test chip

 // make sure it worked (returns 0 if so)
 if (devStatus == 0)
 {
 // turn on the DMP, now that it's ready
 mpu.setDMPEnabled(true);

 // enable Arduino interrupt detection
 attachInterrupt(0, dmpDataReady, RISING);
 mpuIntStatus = mpu.getIntStatus();

 // set our DMP Ready flag so the main loop() function knows it's okay to use it
 dmpReady = true;

 // get expected DMP packet size for later comparison
 packetSize = mpu.dmpGetFIFOPacketSize();
 }
 else
 {
 // ERROR!
 // 1 = initial memory load failed
 // 2 = DMP configuration updates failed
 // (if it's going to break, usually the code will be 1)
 Serial.print(F("DMP Initialization failed (code "));
 Serial.print(devStatus);
 Serial.println(F(")"));
 }
 
 pinMode(13,OUTPUT);
 
pinMode(A0, INPUT_PULLUP);
   pinMode(10, INPUT_PULLUP);
   
   pinMode(s1, INPUT_PULLUP);
   pinMode(s2, INPUT_PULLUP);
   PCintPort::attachInterrupt(A0, encoder1, RISING);
   PCintPort::attachInterrupt(10, encoder2, RISING);
}


void encoder1() {
  if (digitalRead(A0) == HIGH) {
    if (digitalRead(s2) == LOW) {
      en1++;
    }
    else {
      en1--;
    }
  }
  else {
    if (digitalRead(s2) == LOW) {
      en1--;
    } 
    else {
      en1++;
    }
  }
  }


void encoder2() {
      if (digitalRead(10) == HIGH) {
    if (digitalRead(s1) == LOW) {
      en2--;
    }
    else {
      en2++;
    }
  }
  else {
    if (digitalRead(s1) == LOW) {
      en2++;
    } 
    else {
      en2--;
    }
  }
  }
  


void sampleEncoders() {
  static float prevPosition = 0;
  static uint16_t lastUpdate = 0;
  static float leftPosition = 0;
  static float rightPosition = 0;
  //uint16_t m = micros();
  //uint16_t dt3 = m - lastUpdate;
  //lastUpdate = m;

  leftPosition = (float)en1 * 2.32;
  rightPosition = (float)en2 * 2.32;
  motorAngularPosition = -(leftPosition + rightPosition) / 2.0;
  //motorAngularSpeed = (((motorAngularPosition - prevPosition)/ dt3)* 1000000.0)/6;
  //prevPosition = motorAngularPosition;
}


int distancia(){

  disF=en1;
  return disF;
}


float Angle(){
    // if programming failed, don't try to do anything
 if (!dmpReady) return;

 // wait for MPU interrupt or extra packet(s) available
 while (!mpuInterrupt && fifoCount < packetSize)
 {
  //no mpu data - performing lqr calculations and output to motors is 0
 //motorController.move(0,MIN_ABS_SPEED);
 }
 

 // reset interrupt flag and get INT_STATUS byte
 mpuInterrupt = false;
 mpuIntStatus = mpu.getIntStatus();

 // get current FIFO count
 fifoCount = mpu.getFIFOCount();

 // check for overflow (this should never happen unless our code is too inefficient)
 if ((mpuIntStatus & 0x10) || fifoCount == 1024)
 {
 // reset so we can continue cleanly
 mpu.resetFIFO();
 //Serial.println(F("FIFO overflow!"));

 // otherwise, check for DMP data ready interrupt (this should happen frequently)
 }
 else if (mpuIntStatus & 0x02)
 {
 // wait for correct available data length, should be a VERY short wait
 while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

 // read a packet from FIFO
 mpu.getFIFOBytes(fifoBuffer, packetSize);
 
 // track FIFO count here in case there is > 1 packet available
 // (this lets us immediately read more without waiting for an interrupt)
 fifoCount -= packetSize;

 mpu.dmpGetQuaternion(&q, fifoBuffer);
 mpu.dmpGetGravity(&gravity, &q);
 mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
 input = ypr[1] * 180/M_PI + 180;
return input;
 }
}
int now=0;
int chtime=0;
int Ts=10;//20//10
int pasTime=0;

void loop() {///
  now=millis();
  chtime=now-pasTime;
  sampleEncoders();
  x1=motorAngularPosition;
  x3=Angle()-182.5;///sensor izq
  if(chtime>=Ts){ 
  
  
  Serial.print("Desplazamiento: ");
  Serial.print(x1);//*abs(cos(x3))));
  
  x2=((x1 - x1_ant)/ chtime);//(x1-x1_ant)/dt2;
  Serial.print(",  Vel-Desp: ");
  Serial.print(x2);
  
  
  Serial.print(",  Angle: ");
  Serial.print(int(x3));
  
  x4=((x3-x3_ant)/chtime);
  Serial.print(",  Vel-Angle: ");
  Serial.print(x4);
  Serial.print(",,     dt: ");
  Serial.print(chtime);
  
  U=-((k[0]*(SetPoint-x1))+(k[1]*x2)+(k[2]*x3)+(k[3]*x4));
  
  U=constrain(U,-350,350);
  Serial.print("  ,, U: ");
  Serial.print(U);
  U=map(U,-350,350,-120,120);//90vel
  Serial.print("  ,, MOTORES: ");
  Serial.println(U);
  
  motorController.move(U,MIN_ABS_SPEED);
  
  pasTime=now;
  }
  x1_ant=x1;
  x3_ant=x3;
}
