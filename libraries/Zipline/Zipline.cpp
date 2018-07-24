#include "Arduino.h"
#include "Zipline.h"

Zipline::Zipline( int potPin, int echoPin, int trigPin, int liftMotorPin ) {
    // pin numbers
    pinPot = potPin;
    pinEcho = echoPin;
    pinTrig = trigPin;
    pinLiftMotor = liftMotorPin;
    // angle positions
    initialAngle = 90;
    minAngle = 0;
    maxAngle = 270;
    bin1Angle = initialAngle + 45;
    bin2Angle = initialAngle + 90;
    // initial readings
    potReading = 0;
    angle = 0;
    motorSpeed = 100;
}

void Zipline::setDefaultSpeed(int speed) {
    motorSpeed = speed;
}

void Zipline::setMinAngle(int min) {
    minAngle = min;
}

void Zipline::setMaxAngle(int max) {
    maxAngle = max;
}

void Zipline::setBin1Angle(int bin1) {
    bin1Angle = bin1;
}

void Zipline::setBin2Angle(int bin2) {
    bin2Angle = bin2;
}

void Zipline::binLift1() {
    potReading = analogRead(pinPot);
    angle = (potReading / 1000.0) * maxAngle;
    
    while(angle < bin1Angle) {
        motor.speed(pinLiftMotor, motorSpeed);
        potReading = analogRead(pinPot);
        angle = (potReading / 1000.0) * maxAngle;
        
        LCD.clear(); LCD.home();
        LCD.print("Lifting Bin #1");
        LCD.setCursor(0, 1);
        LCD.print(angle);
        LCD.print(" --> ");
        LCD.print(bin1Angle);
    }
    
    motor.speed(pinLiftMotor, 0);
}

void Zipline::binLift2() {
    potReading = analogRead(pinPot);
    angle = (potReading / 1000.0) * maxAngle;
    
    while(angle < bin2Angle) {
        motor.speed(pinLiftMotor, motorSpeed);
        potReading = analogRead(pinPot);
        angle = (potReading / 1000.0) * maxAngle;
        
        LCD.clear(); LCD.home();
        LCD.print("Lifting Bin #2");
        LCD.setCursor(0, 1);
        LCD.print(angle);
        LCD.print(" --> ");
        LCD.print(bin1Angle);
    }
    
    motor.speed(pinLiftMotor, 0);
}


void Zipline::binLower() {
    potReading = analogRead(pinPot);
    angle = (potReading / 1000.0) * maxAngle;
    
    while(angle > initialAngle) {
        motor.speed(pinLiftMotor, -motorSpeed);
        potReading = analogRead(pinPot);
        angle = (potReading / 1000.0) * maxAngle;
        
        LCD.clear(); LCD.home();
        LCD.print("Lowering Bin");
        LCD.setCursor(0, 1);
        LCD.print(angle);
        LCD.print(" --> ");
        LCD.print(initialAngle);
    }
    
    motor.speed(pinLiftMotor, 0);
}




