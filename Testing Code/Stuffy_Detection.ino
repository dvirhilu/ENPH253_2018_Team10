#include <Servo.h>
#include <FrequencyDetection.h>

// pins

constexpr int stuffyComPin = 12;
constexpr int count_limit = 2;
constexpr int edgeComPin = 13;
constexpr int tenkHzPin = 2;
constexpr int onekHzPin = 3;

int pulseLeft = 7;
int pulseRight = 8;
int sensorLeft = 0;
int sensorRight = 1;
int clawPinLeft = 9;
int armPinLeft = 10;
int clawPinRight = 5;
int armPinRight = 6;

// threshold values
int threshold = 150; // IR reading threshold
int clawAngle = 0; // servo starting angle (open position)
int armAngle = 150; // arm servo starting anlge (lifted position)
int onekHzThresh = 50; // change this after testing
int tenkHzThresh = 50; // change this after testing
// initialize variables
int start_time = 0;
int current_time = 0;
int readingHighLeft = 0;
int readingLowLeft = 0;
int readingHighRight = 0;
int readingLowRight = 0;
int countLeft = 0;
int countRight = 0;
int stuffyCount = 0;
Servo clawLeft;
Servo armLeft;
Servo clawRight;
Servo armRight;
FrequencyDetection freq = FrequencyDetection(tenkHzPin, onekHzPin, stuffyComPin);

void setup() {
  Serial.begin(9600);
  pinMode(pulseLeft, OUTPUT);
  pinMode(sensorLeft, INPUT);
  pinMode(pulseRight, OUTPUT);
  pinMode(sensorRight, INPUT);
  pinMode(stuffyComPin, OUTPUT);
  pinMode(edgeComPin, INPUT);
  pinMode(tenkHzPin, INPUT);
  pinMode(onekHzPin, INPUT);
  clawLeft.attach(clawPinLeft);
  armLeft.attach(armPinLeft);
  clawRight.attach(clawPinRight);
  armRight.attach(armPinRight);
}

void loop() {
  digitalWrite(stuffyComPin, LOW);
  clawLeft.write(clawAngle);
  armLeft.write(armAngle);
  clawRight.write(25);
  armRight.write(200 - armAngle);
  delay(1000);
  freq.set1kHzThresh(onekHzThresh);
  freq.set10kHzThresh(tenkHzThresh);
  while (true) {
    start_time = millis();
    current_time = millis();
    if (stuffyCount < 3) {
      digitalWrite(pulseRight, HIGH);
      while (current_time < 5 + start_time) {
        current_time = millis();
        readingHighRight = analogRead(sensorRight);
      }
      digitalWrite(pulseRight, LOW);
      while (current_time < 10 + start_time) {
        current_time = millis();
        readingLowRight = analogRead(sensorRight);
      }
      if (readingHighRight - readingLowRight > threshold) {
        countRight++;
      }
      else {
        countRight = 0;
      }
      if ( countRight == count_limit ) {
        rightDetection();
        countRight = 0;
      }
    }
    else if (digitalRead(edgeComPin) == HIGH) {
      digitalWrite(pulseLeft, HIGH);
      while (current_time < 5 + start_time) {
        current_time = millis();
        readingHighLeft = analogRead(sensorLeft);
      }
      digitalWrite(pulseLeft, LOW);
      while (current_time < 10 + start_time) {
        current_time = millis();
        readingLowLeft = analogRead(sensorLeft);
      }
      if (readingHighLeft - readingLowLeft > threshold) {
        countLeft++;
      }
      else {
        countLeft = 0;
      }
      if ( countLeft == count_limit ) {
        leftDetection();
        countLeft = 0;
      }
    }
  }
}

void leftDetection() {
  digitalWrite(stuffyComPin, HIGH);
  delay(600); // add delay before stopping (sending HIGH to the TINAH)
  Serial.println("detected");
  armLeft.write(20); // change this after testing
  delay(2000);
  clawLeft.write(clawAngle + 28);
  delay(1000);
  armLeft.write(190);
  delay(2000);
  clawLeft.write(clawAngle);
  delay(500);
  armLeft.write(armAngle);
  delay(1000);
  digitalWrite(stuffyComPin, LOW);
}

void rightDetection() {
  if (stuffyCount < 2) {
    digitalWrite(stuffyComPin, HIGH);
    delay(600); // add delay before stopping (sending HIGH to the TINAH)
    Serial.println("detected");
    armRight.write(195); // change this after testing
    delay(2000);
    clawRight.write(clawAngle);
    delay(1000);
    armRight.write(30);
    delay(2000);
    clawRight.write(25);
    delay(500);
    armRight.write(200 - armAngle);
    delay(1000);
    stuffyCount++;
    digitalWrite(stuffyComPin, LOW);
  }
  else if (stuffyCount == 3) {
    digitalWrite(stuffyComPin, HIGH);
    delay(20);
    digitalWrite(stuffyComPin, LOW);
    freq.detectFrequency();
  }
}

