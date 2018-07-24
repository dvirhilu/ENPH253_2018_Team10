#ifndef Zipline_h
#define Zipline_h

#include "Arduino.h"
#include "phys253.h"
#include "LiquidCrystal.h"

class Zipline
{
  public:
    Zipline( int potPin, int echoPin, int trigPin, int liftMotorPin );
    
    void setDefaultSpeed(int speed);
    void setMinAngle(int min);
    void setMaxAngle(int max);
    void setBin1Angle(int bin1);
    void setBin2Angle(int bin2);
    
    
    void binLift1();
    void binLift2();
    void binLower();
    
  private:
    int pinPot;
    int pinEcho;
    int pinTrig;
    int pinLiftMotor;
    int initialAngle;
    int minAngle;
    int maxAngle;
    int bin1Angle;
    int bin2Angle;
    double potReading;
    int angle;
    int motorSpeed;

};

#endif
