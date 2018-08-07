#include "Arduino.h"
#include "Stuffy.h"


Stuffy::Stuffy() {
}

void Stuffy::beginn(int clawL, int armL, int clawR, int armR, int pulseL, int pulseR, int sensorL, int sensorR, int stuffyPin, int edgePin ){
clawLeft.attach(clawL);
  armLeft.attach(armL);
  clawRight.attach(clawR);
  armRight.attach(armR);
  pulseLeft = pulseL;
  pulseRight = pulseR;
  sensorLeft = sensorL;
  sensorRight = sensorR;
  stuffyComPin = stuffyPin;
  edgeComPin = edgePin;
  pinMode(pulseLeft, OUTPUT);
  pinMode(pulseRight, OUTPUT);
  pinMode(sensorLeft, INPUT);
  pinMode(sensorRight, INPUT);
  pinMode(stuffyComPin, OUTPUT);
  pinMode(edgeComPin, INPUT);
  clawAngle = 0; // servo starting angle (open position)
  armAngle = 150; // arm servo starting anlge (lifted position)
  countLeft = 0;
  countRight = 0;
  threshold = 0;
  digitalWrite(stuffyComPin, LOW);
  clawLeft.write(28);
  armLeft.write(armAngle);
  clawRight.write(30);
  armRight.write(200 - armAngle);
  delay(1000);
}

void Stuffy::setSensorThresh( int thresh ) {
  threshold = thresh;
}


bool Stuffy::senseLeft() {
	
  long start_time = millis();
  long current_time = millis();
  int readingHighLeft;
  int readingLowLeft;
  
  digitalWrite(pulseLeft, HIGH);
  while (current_time < 5 + start_time) {
    current_time = millis();
    readingHighLeft = analogRead(sensorLeft);
  }
  digitalWrite(pulseLeft, LOW);
  while (current_time < 10 + start_time) {
    current_time = millis();
    readingLowLeft = analogRead(sensorLeft);
  }
  
  //Serial.println(readingHighLeft - readingLowLeft);
  
  if (readingHighLeft - readingLowLeft > threshold) {
    countLeft++;
  }
  else {
    countLeft = 0;
  }
  if ( countLeft == COUNT_LIMIT ) {
    countLeft = 0;
    return true;
  }
  return false;
}

bool Stuffy::senseRight() {
  long start_time = millis();
  long current_time = millis();
  int readingHighRight;
  int readingLowRight;
  
  digitalWrite(pulseRight, HIGH);
  while (current_time < 5 + start_time) {
    current_time = millis();
    readingHighRight = analogRead(sensorRight);
	//Serial.print("high");
  }
  digitalWrite(pulseRight, LOW);
  while (current_time < 10 + start_time) {
    current_time = millis();
    readingLowRight = analogRead(sensorRight);
	//Serial.println("low");
  }
  
  //Serial.println( readingHighRight - readingLowRight );
  if (readingHighRight - readingLowRight > threshold) {
    countRight++;
  }
  else {
    countRight = 0;
  }
  if ( countRight == COUNT_LIMIT ) {
    countRight = 0;
    return true;
  }
  return false;
}

void Stuffy::rightPickup() {
  digitalWrite(stuffyComPin, HIGH);
  delay(500); // add delay before stopping (sending HIGH to the TINAH)
  Serial.println("detected");
  armRight.write(200); // change this after testing
  delay(100);
  clawRight.write(50);
  delay(400);
  clawRight.write(7);
  delay(300);
  armRight.write(10);
  delay(1000);
  clawRight.write(30);
  digitalWrite(stuffyComPin, LOW);
  delay(200);
  armRight.write(200 - armAngle);
}

void Stuffy::leftPickup() {
  digitalWrite(stuffyComPin, HIGH);
  delay(600); // add delay before stopping (sending HIGH to the TINAH)
  Serial.println("detected");
  armLeft.write(20); // change this after testing
  delay(100);
  clawLeft.write(7);
  delay(800);
  clawLeft.write(54);
  delay(1000);
  armLeft.write(200);
  delay(1000);
  clawLeft.write(28);
  delay(500);
  armLeft.write(armAngle);
  delay(1000);
  digitalWrite(stuffyComPin, LOW);
}

void Stuffy::comeDownSlightly(){
	armLeft.write(100);
}

