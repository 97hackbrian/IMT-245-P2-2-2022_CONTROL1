#include <Fuzzy.h>
   const float Q_distance=2.7;//1 2.7 
   const float R_measure=3.3;//1 2.7
   float distance=0;
   float P=0;
   float K=0;
   float y=0;
   float S=0;
  /////////////////////
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
float motorSpeedFactorLeft = 0.5;
float motorSpeedFactorRight = 0.5;
int ENA = 6;
int IN1 = 7;
int IN2 = 8;
int IN3 = 4;
int IN4 = 5;
int ENB = 3;
#define MIN_ABS_SPEED 10 //27 28 30
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);
////////////////

///encoder///
volatile int en1=0,en2=0;
int motorAngularPosition=0;
float motorAngularSpeed=0;
//const float gearRatio=51.45;
//const float countToDegrees = (360 / (12.0 * gearRatio));
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
//float k[4]={-1.122,-1.68,52.76,719};//-1.22,0.5,30.755,150//-1.22,-0.13,33,100//-1.22,-0.13,37.799,950


// Fuzzy
Fuzzy *fuzzy = new Fuzzy();

// FuzzyInput

/*
// FuzzyDInput
FuzzySet *DBmax = new FuzzySet(-19, -19, -18, -16);
FuzzySet *DBmin = new FuzzySet(-18, -15, -15, -10);
FuzzySet *DStop = new FuzzySet(-10, 0, 0, 10);
FuzzySet *DFmin = new FuzzySet(10, 15, 15, 18);
FuzzySet *DFmax = new FuzzySet(16, 18, 19, 19);
*/
/*
// FuzzyInput
FuzzySet *back = new FuzzySet(-20, -20, -10, -5);
FuzzySet *zero = new FuzzySet(-10, 0, 0, 10);
FuzzySet *forw = new FuzzySet(5, 10, 20, 20);
*/


// FuzzyOutput


int SetPoint=0;
float input;
//double dt=0,dt2=0.0001;
volatile double U;
////////////////////



void setup() {
  //distance = Angle()-182.7;

  FuzzyInput *angle = new FuzzyInput(1);
  FuzzySet *Bmax = new FuzzySet(-50, -40, -29, -2);//13
//FuzzySet *Bmin = new FuzzySet(-8, -5, -5, -1);
FuzzySet *z1 = new FuzzySet(-2, 0, 0, 2);
//FuzzySet *Fmin = new FuzzySet(1, 5, 5, 8);
FuzzySet *Fmax = new FuzzySet(2, 29, 40, 50);
  
  angle->addFuzzySet(Bmax);
  angle->addFuzzySet(z1);
  angle->addFuzzySet(Fmax);
  
  fuzzy->addFuzzyInput(angle);
//////////////////////////////////////////////////////////
FuzzyInput *Dangle = new FuzzyInput(2);
  FuzzySet *DBmax = new FuzzySet(-70, -40, -40, -10);//13
//FuzzySet *Bmin = new FuzzySet(-8, -5, -5, -1);
FuzzySet *Dz1 = new FuzzySet(-10, 0, 0, 10);
//FuzzySet *Fmin = new FuzzySet(1, 5, 5, 8);
FuzzySet *DFmax = new FuzzySet(10, 40, 40, 70);
  Dangle->addFuzzySet(DBmax);
  Dangle->addFuzzySet(Dz1);
  Dangle->addFuzzySet(DFmax);
  fuzzy->addFuzzyInput(Dangle);



  /*
  FuzzyInput *Dangle = new FuzzyInput(2);
  Dangle->addFuzzySet(DBmin);
  Dangle->addFuzzySet(DBmax);
  Dangle->addFuzzySet(DStop);
  Dangle->addFuzzySet(DFmin);
  Dangle->addFuzzySet(DFmax);
  fuzzy->addFuzzyInput(Dangle);
*/
      /*
  FuzzyInput1 *dis = new FuzzyInput(3);
  angle->addFuzzySet(back);
  angle->addFuzzySet(zero);
  angle->addFuzzySet(forw);
  fuzzy->addFuzzyInput(dis);
*/

  // FuzzyOutput
  FuzzyOutput *speedOutput = new FuzzyOutput(1);
  FuzzySet *Bmaximum = new FuzzySet(-120, -120, -37, -1);//-80
//FuzzySet *Bmedium = new FuzzySet(-80, -60, -50, -40);
//FuzzySet *Bless = new FuzzySet(-40, -30, -30, -20);
FuzzySet *nada = new FuzzySet(-1, 0, 0, 1);
//FuzzySet *Fless = new FuzzySet(20, 30, 30, 40);
//FuzzySet *Fmedium = new FuzzySet(40, 50, 60, 80);
FuzzySet *Fmaximum = new FuzzySet(1, 37, 120, 120);//1, 37, 120, 120
  
  
  speedOutput->addFuzzySet(Bmaximum);  
  speedOutput->addFuzzySet(nada);
  speedOutput->addFuzzySet(Fmaximum);
  fuzzy->addFuzzyOutput(speedOutput);


// Building FuzzyRuleS
  FuzzyRuleAntecedent *IfBmaxAndDBmax = new FuzzyRuleAntecedent();
  //IfBmaxAndDBmax->joinSingle(z1);
  IfBmaxAndDBmax->joinWithAND(Bmax,DBmax);

  FuzzyRuleConsequent *ThenBmaxAndDBmax = new FuzzyRuleConsequent();
  ThenBmaxAndDBmax->addOutput(Bmaximum);

  FuzzyRule *fuzzyRule1 = new FuzzyRule(1, IfBmaxAndDBmax, ThenBmaxAndDBmax);
  fuzzy->addFuzzyRule(fuzzyRule1);
///

FuzzyRuleAntecedent *IfBmaxAndDz1 = new FuzzyRuleAntecedent();
  //IfBmaxAndDBmax->joinSingle(z1);
  IfBmaxAndDz1->joinWithAND(Bmax,Dz1);

  FuzzyRuleConsequent *ThenBmaxAndDz1 = new FuzzyRuleConsequent();
  ThenBmaxAndDz1->addOutput(Bmaximum);

  FuzzyRule *fuzzyRule2 = new FuzzyRule(2, IfBmaxAndDz1, ThenBmaxAndDz1);
  fuzzy->addFuzzyRule(fuzzyRule2);
//

FuzzyRuleAntecedent *IfBmaxAndDFmax = new FuzzyRuleAntecedent();
  //IfBmaxAndDBmax->joinSingle(z1);
  IfBmaxAndDFmax->joinWithAND(Bmax,DFmax);

  FuzzyRuleConsequent *ThenBmaxAndDFmax = new FuzzyRuleConsequent();
  ThenBmaxAndDFmax->addOutput(Fmaximum);

  FuzzyRule *fuzzyRule3 = new FuzzyRule(3, IfBmaxAndDFmax, ThenBmaxAndDFmax);
  fuzzy->addFuzzyRule(fuzzyRule3);
//


  FuzzyRuleAntecedent *Ifz1AndDBmax = new FuzzyRuleAntecedent();
  //IfBmaxAndDBmax->joinSingle(z1);
  Ifz1AndDBmax->joinWithAND(z1,DBmax);

  FuzzyRuleConsequent *Thenz1AndDBmax = new FuzzyRuleConsequent();
  Thenz1AndDBmax->addOutput(Bmaximum);

  FuzzyRule *fuzzyRule4 = new FuzzyRule(4, Ifz1AndDBmax, Thenz1AndDBmax);
  fuzzy->addFuzzyRule(fuzzyRule4);
//


  FuzzyRuleAntecedent *Ifz1AndDz1 = new FuzzyRuleAntecedent();
  //IfBmaxAndDBmax->joinSingle(z1);
  Ifz1AndDz1->joinWithAND(z1,Dz1);

  FuzzyRuleConsequent *Thenz1AndDz1 = new FuzzyRuleConsequent();
  Thenz1AndDz1->addOutput(nada);

  FuzzyRule *fuzzyRule5 = new FuzzyRule(5, Ifz1AndDz1,Thenz1AndDz1);
  fuzzy->addFuzzyRule(fuzzyRule5);
//

  FuzzyRuleAntecedent *Ifz1AndDFmax = new FuzzyRuleAntecedent();
  //IfBmaxAndDBmax->joinSingle(z1);
  Ifz1AndDFmax->joinWithAND(z1,DFmax);

  FuzzyRuleConsequent *Thenz1AndDFmax = new FuzzyRuleConsequent();
  Thenz1AndDFmax->addOutput(Fmaximum);

  FuzzyRule *fuzzyRule6 = new FuzzyRule(6, Ifz1AndDFmax,Thenz1AndDFmax);
  fuzzy->addFuzzyRule(fuzzyRule6);
///

  FuzzyRuleAntecedent *IfFmaxAndDBmax = new FuzzyRuleAntecedent();
  //IfBmaxAndDBmax->joinSingle(z1);
  IfFmaxAndDBmax->joinWithAND(Fmax,DBmax);

  FuzzyRuleConsequent *ThenFmaxAndDBmax = new FuzzyRuleConsequent();
  ThenFmaxAndDBmax->addOutput(Bmaximum);

  FuzzyRule *fuzzyRule7 = new FuzzyRule(7, IfFmaxAndDBmax,ThenFmaxAndDBmax);
  fuzzy->addFuzzyRule(fuzzyRule7);


//
  FuzzyRuleAntecedent *IfFmaxAndDz1 = new FuzzyRuleAntecedent();
  //IfBmaxAndDBmax->joinSingle(z1);
  IfFmaxAndDz1->joinWithAND(Fmax,Dz1);

  FuzzyRuleConsequent *ThenFmaxAndDz1 = new FuzzyRuleConsequent();
  ThenFmaxAndDz1->addOutput(Fmaximum);

  FuzzyRule *fuzzyRule8 = new FuzzyRule(8, IfFmaxAndDz1,ThenFmaxAndDz1);
  fuzzy->addFuzzyRule(fuzzyRule8);
//  

  FuzzyRuleAntecedent *IfFmaxAndDFmax = new FuzzyRuleAntecedent();
  //IfBmaxAndDBmax->joinSingle(z1);
  IfFmaxAndDFmax->joinWithAND(Fmax,DFmax);

  FuzzyRuleConsequent *ThenFmaxAndDFmax = new FuzzyRuleConsequent();
  ThenFmaxAndDFmax->addOutput(Fmaximum);

  FuzzyRule *fuzzyRule9 = new FuzzyRule(9, IfFmaxAndDFmax,ThenFmaxAndDFmax);
  fuzzy->addFuzzyRule(fuzzyRule9);
/*
// Building FuzzyRule
  FuzzyRuleAntecedent *Ifnothing = new FuzzyRuleAntecedent();
  Ifnothing->joinSingle(z1);
  //Ifnothing->joinWithAND(Stop, DStop);

  FuzzyRuleConsequent *Thenada = new FuzzyRuleConsequent();
  Thenada->addOutput(nada);

  FuzzyRule *fuzzyRule1 = new FuzzyRule(1, Ifnothing, Thenada);
  fuzzy->addFuzzyRule(fuzzyRule1);
//

// Building FuzzyRule
  FuzzyRuleAntecedent *Ifbmax = new FuzzyRuleAntecedent();
  Ifbmax->joinSingle(Bmax);
  //Ifnothing->joinWithAND(Stop, DStop);

  FuzzyRuleConsequent *Thenbmax = new FuzzyRuleConsequent();
  Thenbmax->addOutput(Bmaximum);

  FuzzyRule *fuzzyRule2 = new FuzzyRule(2, Ifbmax, Thenbmax);
  fuzzy->addFuzzyRule(fuzzyRule2);
  
//

// Building FuzzyRule
  FuzzyRuleAntecedent *Iffmax = new FuzzyRuleAntecedent();
  
  Iffmax->joinSingle(Fmax);
  //Ifnothing->joinWithAND(Stop, DStop);

  FuzzyRuleConsequent *Thenfmax = new FuzzyRuleConsequent();
  Thenfmax->addOutput(Fmaximum);

  FuzzyRule *fuzzyRule3 = new FuzzyRule(3, Iffmax, Thenfmax);
  fuzzy->addFuzzyRule(fuzzyRule3);
//
*/
  
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
int Ts=1;//20//10
int pasTime=0;





/*
float KalmanFilter(float newDistance, float dt){
       //distance=distance; //priori distance
       P+=Q_distance*dt;  //estimation error covariance
       //Kalman gain
       S=P+R_measure;
       K=P/S;
       //Update whith measurement 
       y=newDistance-distance;
       //Calculate distance
       distance+=K*y;
       //Update the error covariance
       P*=(1-K);
       return distance;
   }
*/


void loop() {///
  x3=Angle()-182.7;///sensor izq 182.7
  x3=constrain(x3,-30,30);
  
  fuzzy->setInput(1, int(x3));
  fuzzy->setInput(2, int(x4));
  //fuzzy->setInput(2, x4);
  fuzzy->fuzzify();
  U =int( fuzzy->defuzzify(1));
  
  now=millis();
  chtime=now-pasTime;
  //sampleEncoders();
  //x1=motorAngularPosition;  
  delay(3);//5//3
  
  

  //Serial.print(int(KalmanFilter(x3,chtime*10000)));
  //Serial.print(' ');
  //Serial.println(KalmanFilter(x3,chtime*1000000000));
  Serial.print(",  Angle: ");
  Serial.print(x3);
  Serial.print(",  VelAngle: ");
  Serial.print(int(x4));
  
  x4=((x3-x3_ant)/chtime)*50;
  x4=constrain(x4,-50,50);
  
  
  //Serial.print(",  Vel-Angle: ");
  //Serial.print(x4);
  //Serial.print(",,     dt: ");
  //Serial.print(chtime);
  ///////////////////////////////////////////////////////////////////////////////////////
  //U=-((k[0]*(SetPoint-x1))+(k[1]*x2)+(k[2]*KalmanFilter(x3,chtime*1000000))+(k[3]*x4));
  ///////////////////////////////////////////////////////////////////////////////////////
  
  //U=constrain(U,-120,120);
  U=constrain(U,-120,120);
  Serial.print("  ,, U: ");
  Serial.print(U);
  //U=map(U,-70,70,-90,90);//90vel
  Serial.print("  ,, MOTORES: ");
  Serial.println(U);
  motorController.move(int(U)*-1,MIN_ABS_SPEED);
  pasTime=now;
  
  x1_ant=x1;
  x3_ant=x3;
}
