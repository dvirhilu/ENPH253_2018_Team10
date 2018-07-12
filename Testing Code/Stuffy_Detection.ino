#include <Servo.h>
int pulse = 7;
int switchPin = 8;
int start_time = 0;
int current_time = 0;
int threshold = 400;
int sensor = 0;
int readingHigh = 0;
int readingLow = 0;
int clawPin = 9;
int servoAngle = 0;
Servo claw;

void setup() {
  Serial.begin(9600);
  pinMode(pulse, OUTPUT);
  pinMode(sensor, INPUT);
  pinMode(switchPin, INPUT);
  claw.attach(clawPin);
}

void loop() {
  claw.write(35);
  servoAngle = claw.read();
  //threshold = menuToggle();
  //while (!startbutton()) {
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
    Serial.println("detected");
    claw.write(servoAngle - 20);
    while (true) {
      if (digitalRead(switchPin) == 0) {
        Serial.println("captured");
        break;
      }
    }
    delay(3000);
    claw.write(servoAngle);
  }
}
