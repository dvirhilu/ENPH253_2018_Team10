/*
  BinLift.h - Library for bin lifting
*/

#ifndef BinLift_h
#define BinLift_h

#include "Arduino.h"
#include "BinState.h"

class BinLift
{
  public:
    BinLift(int motor,int QRD, int thresh, int encoder);
    void raise();
	void lower();
	void poll();
  private:
    int motorPin;
	int QRDPin;
	int encoderPin;
	int QRDthresh;
	BinState state;
};

#endif
