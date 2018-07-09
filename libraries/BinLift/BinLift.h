/*
  BinLift.h - Library for bin lifting
*/

#ifndef BinLift_h
#define BinLift_h

#include "Arduino.h"
#include "BinState.h"
#include "NewPing.h"

class BinLift
{
  public:
    BinLift(int motor,int sonar, int thresh, int encoder);
    void raise();
	void lower();
	void poll();
  private:
    int motorPin;
	int sonarPin;
	int encoderPin;
	int sonarThresh;
	BinState state;
};

#endif
