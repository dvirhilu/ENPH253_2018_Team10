/*
  StuffyRescue.h - Library for stuffy detection and pickup
*/

#ifndef StuffyRescue_h
#define StuffyRescue_h

#include "Arduino.h"
#include "Servo.h"
#include "StuffyState.h"

class StuffyRescue
{
  public:
    StuffyRescue(int limitSwitch, int arm, int claw, int sensor, int LED);
    void setup(int armPos, int clawPos); // sets up arm and claw to the setup position and changes StuffyState to search
    void poll(int thresh); //changes StuffyState if sensor reading becomes above threshold
	void pickup(int closedPos, int loweredPos, int armInitialPos, int clawInitialPos); // picks up and stores stuffy
  private:
    int switchPin;
	int armServoPin;
	int clawServoPin;
	int sensorPin;
	int LEDPin;
	Servo armServo;
	Servo clawServo;
	StuffyState state;
};

#endif
