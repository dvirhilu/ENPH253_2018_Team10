#include <Servo.h>
#include <FrequencyDetection.h>
#include <Stuffy.h>

// pins
constexpr int stuffyComPin = 4;
constexpr int count_limit = 2;
constexpr int edgeComPin = 12;
constexpr int tenkHzPin = 3;
constexpr int onekHzPin = 2;
constexpr int pulseLeft = 7;
constexpr int pulseRight = 8;
constexpr int sensorLeft = 0;
constexpr int sensorRight = 1;

constexpr int clawPinLeft = 9;
constexpr int armPinLeft = 10;
constexpr int clawPinRight = 5;
constexpr int armPinRight = 6;

// threshold values
int threshold = 200; // IR reading threshold
int clawAngle = 0; // servo starting angle (open position)
int armAngle = 150; // arm servo starting anlge (lifted position)
int onekHzThresh = 100; // change this after testing
int tenkHzThresh = 40; // change this after testing
int stuffyCount = 0;
bool freqDone = false;
bool binDone = false;

// initialize variables
Stuffy stuffy = Stuffy();
FrequencyDetection freq = FrequencyDetection(tenkHzPin, onekHzPin, stuffyComPin);

void setup() {
  Serial.begin(9600);
  Serial.print("begin");
  stuffy.beginn(clawPinLeft, armPinLeft, clawPinRight, armPinRight, pulseLeft, pulseRight, sensorLeft, sensorRight, stuffyComPin, edgeComPin);
  pinMode(LED_BUILTIN, OUTPUT);
  freq.set1kHzThresh(onekHzThresh);
  freq.set10kHzThresh(tenkHzThresh);
  stuffy.setSensorThresh(threshold);
  digitalWrite(stuffyComPin, LOW);
  }

void loop() {


  if( stuffyCount == 0 && stuffy.senseRight() && freqDone == false){
    stuffy.rightPickup();
    stuffyCount++;
    Serial.println("first stuffy");
    //first stuffy
  }
  else if( stuffyCount == 1 && debounceEdgeComPin() && freqDone == false){
    freq.detectFrequency();
    stuffyCount++;
    freqDone == true;
    Serial.println("IR");
    // IR
  }
  else if( stuffyCount == 2 && stuffy.senseRight() ){
    stuffy.rightPickup();
    stuffyCount++;
    Serial.println("second stuffy");
    //second stuffy
  }
  else if( stuffyCount == 3 && stuffy.senseRight() ){
    digitalWrite( stuffyComPin, HIGH );
    stuffyCount++;
    delay(500);
    digitalWrite( stuffyComPin, LOW );
    Serial.println("arch");
    //archway
  }
  else if( stuffyCount == 4 && stuffy.senseRight() ){
    digitalWrite( stuffyComPin, HIGH );
    stuffyCount++;
    delay(200);
    digitalWrite( stuffyComPin, LOW );
    Serial.println("storm trooper");
    //storm trooper
  }
  else if( stuffyCount >= 5 && debounceEdgeComPin() && stuffy.senseLeft() ){
    stuffy.leftPickup();
    stuffyCount++;
    Serial.println("left stuffy");
    //left stuffies
  }
  
  /*
  if (stuffyCount < 4) {
    if (digitalRead(edgeComPin) == HIGH && freqDone == false) {
      Serial.println("I'm a freq");
      freq.detectFrequency();
      freqDone = true;
      Serial.println("what the hell am I doing here");
    }
    else if (stuffy.senseRight()) {
      if (stuffyCount < 2) {
        Serial.println("stuffy");
        digitalWrite(LED_BUILTIN, HIGH);
        stuffy.rightPickup();
        digitalWrite(LED_BUILTIN, LOW);
        stuffyCount++;
      }
      else if( stuffyCount == 2){
        Serial.println("arch");
        stuffyCount++;
        delay(700);
      }
      else if (stuffyCount == 3) {
        Serial.println("stormTrooper");
        digitalWrite(stuffyComPin, LOW);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(stuffyComPin, HIGH);
        digitalWrite(LED_BUILTIN, LOW);
        stuffyCount++;
        delay(1000);
        stuffy.comeDownSlightly();
      }
    }
  }
  else if ( digitalRead(edgeComPin) == HIGH ) {
    if (stuffy.senseLeft()) {
      digitalWrite(LED_BUILTIN, HIGH);
      stuffy.leftPickup();
      digitalWrite(LED_BUILTIN, LOW);
    }
  }*/
}

bool debounceEdgeComPin(){
  if( digitalRead(edgeComPin) == HIGH ){
    delay(10);
    if( digitalRead(edgeComPin) == HIGH){
      return true;
    }
  }

  return false;
}

