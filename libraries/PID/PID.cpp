#include "Arduino.h"
#include "PID.h"

PID::PID( int leftPin, int rightPin, int leftMotorPin, int rightMotorPin ) {
  pinLeft = leftPin;
  pinRight = rightPin;
  motorLeft = leftMotorPin;
  motorRight' = rightMotorPin;
  leftDark = 0;
  rightDark = 0;
  default_speed = 0;
  gain = 0;
  k_p = 0;
  k_d = 0;
  prev_time = 0;
  prev_error = 0;
}

void PID::initizlize(){
  prev_time = micros()/1000000.0;
}

  void PID::setKp( int kp ) {
  k_p = kp;
}

  void PID::setKd( int kd ) {
  k_d = kd;
}

  void PID::setGain( int gainz ) {
  gain = gainz;
}

  void PID::setLeftDark( int lDark ) {
  leftDark = lDark;
}

  void PID::setRightDark( int rDark ) {
  rightDark = rDark;
}

  void PID::setDefaultSpeed( int motorSpeed ) {
  default_speed = motorSpeed;
}

  double PID::double getPID(){
  int sensorLeft = analogRead(sensorLeftPin);
  int sensorRight = analogRead(sensorRightPin);
  double current_time = micros()/1000000.0;
  double error = 0;
  double derivative = 0;
  double pid;

  LCD.clear();
  LCD.setCursor(0, 0);
  LCD.print((int)sensorLeft + String(" ") + (int)sensorRight + String(" ") + error + String(" ") + derivative);

  LCD.setCursor(0, 1);


  //error = (sensorRight - rightDark) + (leftDark - sensorLeft);
  if (sensorRight < rightDark && !(sensorLeft < leftDark)) {
  leftError = false;
  error =  rightDark - sensorRight;
}
  else if ( !(sensorRight < rightDark) && sensorLeft < leftDark) {
  leftError = true;
  error = sensorLeft - leftDark;
}
  else if (sensorRight < rightDark && sensorLeft < leftDark) {
  if (leftError) {
  error = (sensorRight - rightDark) + (sensorLeft - leftDark) ;
}
  else {
  error = (rightDark - sensorRight) + (leftDark - sensorLeft);
}
}
  else {
  error = 0;
}

  derivative = (error - prev_error) / (current_time - prev_time);

  pid = gain * (k_p / 10.0 * error + k_d / 10.0 * derivative);

  prev_error = error;
  prev_time = current_time;

  return pid;
}

  void PID::tapeFollow() {

  double pid = getPID();

  if ( pid < 0 ) {
  motor.speed(motorLeft, default_speed - 0.5 * pid);
  motor.speed(motorRight, default_speed + 0.5 * pid);
  LCD.print((default_speed - pid) + String(" ") + default_speed);

}
  else {
  motor.speed(motorLeft, default_speed - 0.5 * pid);
  motor.speed(motorRight, default_speed + 0.5 *
  pid);
  LCD.print((default_speed) + String(" ") + (default_speed + pid));
}
  delay(50);
}

