#include <Servo.h>
// pins

constexpr int stuffyComPin = 11;

int pulse = 7;
int switchPin = 8;
int sensor = 0;
int clawPin = 9;
int armPin = 10;
// threshold values
int threshold = 400; // IR reading threshold
int clawAngle = 0; // servo starting angle (open position)
int armAngle = 140; // arm servo starting anlge (lifted position)
// initialize variables
int start_time = 0;
int current_time = 0;
int readingHigh = 0;
int readingLow = 0;
Servo claw;
Servo arm;

void setup() {
  Serial.begin(9600);
  pinMode(pulse, OUTPUT);
  pinMode(sensor, INPUT);
  pinMode(switchPin, INPUT);
  pinMode(stuffyComPin, OUTPUT);
  claw.attach(clawPin);
  arm.attach(armPin);
}

void loop() {
  claw.write(clawAngle);
  arm.write(armAngle);
  delay(100);
  start_time = millis();
  current_time = millis();
  digitalWrite(pulse, HIGH);
  while (current_time < 100 + start_time) {
    current_time = millis();
    readingHigh = analogRead(sensor);
  }
  digitalWrite(pulse, LOW);
  while (current_time < 200 + start_time) {
    current_time = millis();
    readingLow = analogRead(sensor);
  }
  Serial.println(readingHigh + String(" ") + readingLow + String(" ") + (readingHigh - readingLow));
  if (readingHigh - readingLow > threshold) {
    digitalWrite(stuffyComPin, HIGH);
    Serial.println("detected");
    arm.write(20); // change this after testing
    /*
      while (current_time < 400 + start_time) {
      current_time = millis();
      }
    */
    delay(2000);
    claw.write(clawAngle + 28);
    while (true) {
      if (digitalRead(switchPin) == 0) {
        Serial.println("captured");
        break;
      }
    }
    arm.write(200);
    /*while (current_time < 1000 + start_time) {
      current_time = millis();
      }*/
    delay(2000);
    claw.write(clawAngle);
    arm.write(armAngle);
    delay(500);
    digitalWrite(stuffyComPin, LOW);
  }
}
