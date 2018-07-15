#ifndef PID_h
#define PID_h

#include "Arduino.h"

class PID
{
  public:
    PID( int leftPin, int rightPin, int leftMotorPin, int rightMotorPin );
    void setKd( int kd );
    void setKp( int kp);
    void setGain( int gainz );
    void setLeftDark( int lDark );
    void setRightDark( int rDark );
    void setDefaultSpeed( int motorSpeed );
    double getPID();
    void tapeFollow();
  private:
    int pinLeft;
    int pinRight;
    int motorLeft;
    int motorRight;
    int leftDark;
    int rightDark;
    int default_speed;
    int gain;
    int k_p;
    int k_d;
    //double current_time;
    double prev_time;
    //double error = 0;
    double prev_error;
    //int derivative = 0;
    //double pid;
    //boolean leftError = true;
};

#endif
