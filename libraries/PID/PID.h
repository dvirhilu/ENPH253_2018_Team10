#ifndef PID_h
#define PID_h

#include "Arduino.h"
#include "phys253.h"
#include "LiquidCrystal.h"

class PID
{
  public:
    PID( int leftPin, int rightPin, int leftMotorPin, int rightMotorPin );
    void initialize();
    void setKp( int kp );
    void setKd( int kd );
    void setGain( int gainz );
    void setLeftDark( int lDark );
    void setRightDark( int rDark );
    void setDefaultSpeed( int motorSpeed );
	void setRatio(int percentage);
	void setEdgeThresh( int thresh );
    double getPID();
    void tapeFollow();
	bool isEdge();
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
	int ratio;
	int edgeThresh;
    double prev_time;
    double prev_error;
    boolean leftError;
};

#endif
