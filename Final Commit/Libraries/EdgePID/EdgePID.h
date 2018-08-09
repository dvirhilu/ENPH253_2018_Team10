#ifndef EdgePID_h
#define EdgePID_h

#include "Arduino.h"
#include "phys253.h"
#include "LiquidCrystal.h"

#define LEFT_THRESH 300
#define LEFT_REKT 400
#define RIGHT_THRESH 200
#define RIGHT_REKT 100

class EdgePID
{
  public:
    EdgePID( int leftPin, int rightPin, int leftMotorPin, int rightMotorPin );
    void setEdgeKp( int kp );
    void setDefaultSpeed( int motorSpeed );
    double getEdgePID();
    void edgeFollow();
  private:
    int pinLeft;
    int pinRight;
    int motorLeft;
    int motorRight;
    int default_speed;
    int k_p;
};

#endif
