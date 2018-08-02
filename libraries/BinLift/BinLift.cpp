#include "Arduino.h"
#include "BinLift.h"

BinLift::BinLift(int liftMotorPin) {
    // pin numbers
    pinLiftMotor = liftMotorPin;
    
    // initial readings
    raisingMotorSpeed = 254;
    loweringMotorSpeed = 75;
}

void BinLift::setDefaultSpeed(int lift_speed, int lower_speed) {
    raisingMotorSpeed = lift_speed;
    loweringMotorSpeed = lower_speed;
    
}

void BinLift::binLift() {
    motor.speed(pinLiftMotor, raisingMotorSpeed);
}

void BinLift::binLower() {
    motor.speed(pinLiftMotor, loweringMotorSpeed);
}

