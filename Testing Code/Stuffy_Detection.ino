#include <Servo.h>
// pins

constexpr int stuffyComPin = 12;

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
// initialize variables
int start_time = 0;
int current_time = 0;
int readingHighLeft = 0;
int readingLowLeft = 0;
int readingHighRight = 0;
int readingLowRight = 0;
int countLeft = 0;
int countRight = 0;
Servo clawLeft;
Servo armLeft;
Servo clawRight;
Servo armRight;

void setup() {
  Serial.begin(9600);
  pinMode(pulseLeft, OUTPUT);
  pinMode(sensorLeft, INPUT);
  pinMode(pulseRight, OUTPUT);
  pinMode(sensorRight, INPUT);
  pinMode(stuffyComPin, OUTPUT);
  clawLeft.attach(clawPinLeft);
  armLeft.attach(armPinLeft);
  clawRight.attach(clawPinRight);
  armRight.attach(armPinRight);
}

void loop() {
  digitalWrite(stuffyComPin, LOW);
  clawLeft.write(clawAngle);
  armLeft.write(armAngle);
  clawRight.write(clawAngle);
  armRight.write(armAngle);
  delay(1000);
  while (true) {
    start_time = millis();
    current_time = millis();
    digitalWrite(pulseLeft, HIGH);
    digitalWrite(pulseRight, HIGH);
    while (current_time < 50 + start_time) {
      current_time = millis();
      readingHighLeft = analogRead(sensorLeft);
      readingHighRight = analogRead(sensorRight);
    }
    digitalWrite(pulseLeft, LOW);
    digitalWrite(pulseRight, LOW);
    while (current_time < 100 + start_time) {
      current_time = millis();
      readingLowLeft = analogRead(sensorLeft);
      readingLowRight = analogRead(sensorRight);
    }
    //Serial.println(readingHigh + String(" ") + readingLow + String(" ") + (readingHigh - readingLow));
    if (readingHighLeft - readingLowLeft > threshold) {
      countLeft++;
    }
    else {
      countLeft = 0;
    }
    if (readingHighRight - readingLowRight > threshold) {
      countRight++;
    }
    else {
      countRight = 0;
    }
    if ( countLeft > 3 ) {
      leftDetection();
      countLeft = 0;
    }
    if ( countRight > 3 ) {
      rightDetection();
      countRight = 0;
    }
  }
}

void leftDetection() {
  delay(600); // add delay before stopping (sending HIGH to the TINAH)
  digitalWrite(stuffyComPin, HIGH);
  Serial.println("detected");
  armLeft.write(20); // change this after testing
  /*
    while (current_time < 400 + start_time) {
    current_time = millis();
    }
  */
  delay(2000);
  clawLeft.write(clawAngle + 28);
  delay(1000);
  /*
    while (true) {
    if (digitalRead(switchPin) == 0) {
      Serial.println("captured");
      break;
    }
    }
  */
  armLeft.write(200);
  /*while (current_time < 1000 + start_time) {
    current_time = millis();
    }*/
  delay(2000);
  clawLeft.write(clawAngle);
  delay(500);
  armLeft.write(armAngle);
  delay(1000);
  digitalWrite(stuffyComPin, LOW);
}


void rightDetection() {
  delay(600); // add delay before stopping (sending HIGH to the TINAH)
  digitalWrite(stuffyComPin, HIGH);
  Serial.println("detected");
  armRight.write(20); // change this after testing
  /*
    while (current_time < 400 + start_time) {
    current_time = millis();
    }
  */
  delay(2000);
  clawRight.write(clawAngle + 28);
  delay(1000);
  /*
    while (true) {
    if (digitalRead(switchPin) == 0) {
      Serial.println("captured");
      break;
    }
    }
  */
  armRight.write(200);
  /*while (current_time < 1000 + start_time) {
    current_time = millis();
    }*/
  delay(2000);
  clawRight.write(clawAngle);
  delay(500);
  armRight.write(armAngle);
  delay(1000);
  digitalWrite(stuffyComPin, LOW);
}

