#include "Arduino.h"
#include "PlaceBridge1.h"
#include "phys253.h"

PlaceBridge1::PlaceBridge1(int motorLeft,int motorRight, int angle){
    motorLeftPin = motorLeft;
    motorRightPin = motorRight;
    anglePosition = angle;
    
}

void PlaceBridge1::process() {
    // back up at default speed for a short distance
    motor.speed(motorLeftPin, -100);
    motor.speed(motorRightPin, -100);
    delay(50);
    motor.stop_all();
    
    // drop the first bridge
    RCServo0.write(angle_Position);
    delay(1000);
    
    // back up at default speed for a short distance
    motor.speed(motorLeftPin, -100);
    motor.speed(motorRightPin, -100);
    delay(50);
    motor.stop_all();
    delay(500);
    
    // move forward and start tape following as soon as the tape is detected
    motor.speed(motorLeftPin, 100);
    motor.speed(motorRightPin, 100);
    return;
}




