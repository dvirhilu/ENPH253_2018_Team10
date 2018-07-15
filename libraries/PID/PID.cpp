#include "Arduino.h"
#include "PID.h"

PID::PID( int leftPin, int rightPin, int leftMotorPin, int rightMotorPin ) {
  pinLeft;
  pinRight;
  motorLeft;
  motorRight;
  leftDark;
  rightDark;
  default_speed;
  gain;
  k_p;
  k_d;
  prev_time;
  prev_error;
}

const char* PID::getName() {
  return itemName;
}

int PID::getValue() {
  return eeprom_read_word(EEPROMAddress);
}
void PID::setValue( int value ) {
  eeprom_write_word(EEPROMAddress, value);
}

int PID::getEepNum() {
  return (int)EEPROMAddress;
}
