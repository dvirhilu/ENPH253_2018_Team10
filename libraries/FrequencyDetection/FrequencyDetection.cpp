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
	Serial.print( analogRead(pin10kHz) + String(" 10K ") );
  if ( analogRead(pin10kHz) > thresh10kHz ) {
    return  true;
  }
  return false;
}

bool FrequencyDetection::is1kHz() {
	Serial.print(analogRead(pin1kHz) + String(" 1K "));
  if ( analogRead(pin1kHz) > thresh1kHz ) {
    return  true;
  }
  return false;
}

void FrequencyDetection::handle10kHz() {
 
while ( !(is1kHz()) ) { digitalWrite(LED_BUILTIN, HIGH); }
digitalWrite(LED_BUILTIN, LOW);
  while ( !(is10kHz()) ) {}
}

void FrequencyDetection::handle1kHz() {
  
  while ( !(is10kHz()) ) { digitalWrite(LED_BUILTIN, HIGH);}
  digitalWrite(LED_BUILTIN,LOW);
}

void FrequencyDetection::detectFrequency() {
  digitalWrite(tinahComPin, HIGH);
  while (true) {
    if ( is1kHz() ) {
		Serial.println("it's 1K");
      handle1kHz();
	  digitalWrite(tinahComPin, LOW );
      break;
    }
    else if ( is10kHz() ) {
		Serial.println("it's 10K");
      handle10kHz();
	  digitalWrite(tinahComPin, LOW );
      break;
    }
  }
}
