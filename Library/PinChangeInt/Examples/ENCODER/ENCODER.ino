#include <PinChangeInt.h>

#define GREENLED  13
volatile int a=0,b=0;
int GradosP=72;
#define s1 11
#define s2 12

void setup(){
  Serial.begin(2000000);
  pinMode(GREENLED,OUTPUT);
  
  
   pinMode(A0, INPUT_PULLUP);
   pinMode(10, INPUT_PULLUP);
   
   pinMode(s1, INPUT_PULLUP);
   pinMode(s2, INPUT_PULLUP);
   
   PCintPort::attachInterrupt(A0, isr, RISING);
   PCintPort::attachInterrupt(10, isr2, RISING);
}

void loop(){
  Serial.print("encoderA: ");
  Serial.print((a)*2.32);
  Serial.print(" , encoderB: ");
  Serial.println((b)*2.32);
  digitalWrite(GREENLED,0);
}

void isr2() {
    if (digitalRead(10) == HIGH) {
    if (digitalRead(s1) == LOW) {
      b--;
    }
    else {
      b++;
    }
  }
  else {
    if (digitalRead(s1) == LOW) {
      b++;
    } 
    else {
      b--;
    }
  }
}

void isr() {
  if (digitalRead(A0) == HIGH) {
    if (digitalRead(s2) == LOW) {
      a++;
    }
    else {
      a--;
    }
  }
  else {
    if (digitalRead(s2) == LOW) {
      a--;
    } 
    else {
      a++;
    }
  }
}
