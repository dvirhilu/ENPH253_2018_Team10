/*
 PlaceBridge1.h - Library for bridge placing
 */

#ifndef PlaceBridge1_h
#define PlaceBridge1_h

#include "Arduino.h"
#include "BridgeState1.h"

class PlaceBridge1
{
public:
    PlaceBridge1(int motorLeft,int motorRight, int bridgeServo, int angle);
    void raise();
    void lower();
    void poll();
private:
    int motorLeftPin;
    int motorRightPin;
    int bridgeServoPin;
    int anglePosition;
    BridgeState state;
};
e
#endif
