#ifndef Stuffy_h
#define Stuffy_h

#include "Arduino.h"
#include "Servo.h"

#define COUNT_LIMIT 2

class Stuffy
{
  public:
    Stuffy();
	void beginn(int clawL, int armL, int clawR, int armR, int pulseL, int pulseR, int sensorL, int sensorR, int stuffyPin, int edgePin );
    void setSensorThresh( int thresh );
    bool senseLeft();
    bool senseRight();
    void rightPickup();
	void leftPickup();
  private:
    Servo clawLeft;
    Servo armLeft;
    Servo clawRight;
    Servo armRight;
    int pulseLeft;
    int pulseRight;
    int sensorLeft;
    int sensorRight;
    int stuffyComPin;
    int edgeComPin;
    int threshold;
	int clawAngle;
	int armAngle;
	int countLeft;
	int countRight;
};

#endif

