#include <Servo.h>
#include <FrequencyDetection.h>
#include <Stuffy.h>

// pins
constexpr int stuffyComPin = 12;
constexpr int count_limit = 2;
constexpr int edgeComPin = 13;
constexpr int tenkHzPin = 2;
constexpr int onekHzPin = 3;
constexpr int pulseLeft = 7;
constexpr int pulseRight = 8;
constexpr int sensorLeft = 0;
constexpr int sensorRight = 1;
constexpr int clawPinLeft = 9;
constexpr int armPinLeft = 10;
constexpr int clawPinRight = 5;
constexpr int armPinRight = 6;

// threshold values
int threshold = 150; // IR reading threshold
int clawAngle = 0; // servo starting angle (open position)
int armAngle = 150; // arm servo starting anlge (lifted position)
int onekHzThresh = 50; // change this after testing
int tenkHzThresh = 50; // change this after testing
int stuffyCount = 0;

// initialize variables
Stuffy stuffy = Stuffy();
//FrequencyDetection freq = FrequencyDetection(tenkHzPin, onekHzPin, stuffyComPin);

void setup() {
  Serial.begin(9600);
  Serial.print("begin");
  stuffy.beginn(clawPinLeft, armPinLeft, clawPinRight, armPinRight, pulseLeft, pulseRight, sensorLeft, sensorRight, stuffyComPin, edgeComPin);
}

void loop() {

  Serial.print("0");
  //freq.set1kHzThresh(onekHzThresh);
  //freq.set10kHzThresh(tenkHzThresh);
  stuffy.setSensorThresh(threshold);
  while (true) {
    if (stuffyCount < 3) {
      Serial.println("1");
      if (stuffy.senseRight()) {
        Serial.println("2");
        if (stuffyCount < 2) {
          Serial.print("3");
          stuffy.rightPickup();
          stuffyCount++;
          if (stuffyCount == 2) {
            //freq.detectFrequency();
          }
        }
        else if (stuffyCount == 2) {
          digitalWrite(stuffyComPin, HIGH);
          delay(20);
          digitalWrite(stuffyComPin, LOW);
          stuffyCount++;
        }
      }
    }
    else if (digitalRead(edgeComPin) == HIGH) {
      if (stuffy.senseLeft()) {
        stuffy.leftPickup();
      }
    }
  }
}
