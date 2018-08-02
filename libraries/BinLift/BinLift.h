#ifndef BinLift_h
#define BinLift_h

#include "Arduino.h"
#include "phys253.h"
#include "LiquidCrystal.h"

class BinLift
{
  public:
    BinLift(int liftMotorPin);
    
    void setDefaultSpeed(int lift_speed, int lower_speed);

    void binLift();
    void binLower();
    
  private:
    int pinLiftMotor;
    int raisingMotorSpeed;
    int loweringMotorSpeed;

};

#endif
