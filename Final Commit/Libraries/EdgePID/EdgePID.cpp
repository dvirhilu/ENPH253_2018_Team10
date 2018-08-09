#include "Arduino.h"
#include "EdgePID.h"

EdgePID::EdgePID( int leftPin, int rightPin, int leftMotorPin, int rightMotorPin ) {
  pinLeft = leftPin;
  pinRight = rightPin;
  motorLeft = leftMotorPin;
  motorRight = rightMotorPin;
  default_speed = 0;
  k_p = 0;
}
void EdgePID::setEdgeKp( int kp ) {
  k_p = kp;
}

void EdgePID::setDefaultSpeed( int motorSpeed ) {
  default_speed = motorSpeed;
}

double EdgePID::getEdgePID() {
  int sensorLeft = analogRead(pinLeft);
  int sensorRight = analogRead(pinRight);
  double error = 0;
  double pid;

  if ( sensorLeft > LEFT_REKT ) {
    error = 3;
  }
  else if ( sensorRight < RIGHT_REKT ) {
    error = -3;
  }
  else if ( sensorLeft > LEFT_THRESH ) {
    error = 1;
  }
  else if ( sensorRight < RIGHT_THRESH ) {
    error = -1;
  }
  else {
    error = 0;
  }

  pid = k_p * error;

  return pid;
}

void EdgePID::edgeFollow() {

  double pid = getEdgePID();

  motor.speed(motorLeft, default_speed - 0.5 * pid);
  motor.speed(motorRight, default_speed + 0.5 * pid);
}

