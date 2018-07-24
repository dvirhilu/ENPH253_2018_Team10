#include "Arduino.h"
#include "FrequencyDetection.h"


FrequencyDetection::FrequencyDetection( int tenkHz, int onekHz, int tinahPin ) {
  pin10kHz = tenkHz;
  pin1kHz = onekHz;
  tinahComPin = tinahPin;
  pinMode(pin10kHz, INPUT);
  pinMode(pin1kHz, INPUT);
  pinMode(tinahComPin, OUTPUT);
}

void FrequencyDetection::set10kHzThresh( int thresh ) {
  thresh10kHz = thresh;
}

void FrequencyDetection::set1kHzThresh( int thresh ) {
  thresh1kHz = thresh;
}

bool FrequencyDetection::is10kHz() {
  if ( analogRead(pin10kHz) > thresh10kHz ) {
    return  true;
  }
  return false;
}

bool FrequencyDetection::is1kHz() {
  if ( analogRead(pin1kHz) > thresh1kHz ) {
    return  true;
  }
  return false;
}

void FrequencyDetection::handle10kHz() {
  digitalWrite(tinahComPin, HIGH );
  while ( !(is1kHz()) ) {}
  while ( !(is10kHz()) ) {}
  digitalWrite(tinahComPin, LOW );
}

void FrequencyDetection::handle1kHz() {
  digitalWrite(tinahComPin, HIGH);
  while ( !(is10kHz()) ) {}
  digitalWrite(tinahComPin, LOW );
}

void FrequencyDetection::detectFreqency() {
  while (true) {
    if ( is1kHz() ) {
      handle1kHz();
      break;
    }
    else if ( is10kHz() ) {
      handle10kHz();
      break;
    }
  }
}
