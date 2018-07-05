/*
  Robot.h - Library for controlling the action of the robot.
*/

#ifndef Robot_h
#define Robot_h

#include "Arduino.h"

class Robot
{
  public:
    Robot();
    bool initialization();
    void pickup();
    void displayLCD(char* message);
    
};

#endif
