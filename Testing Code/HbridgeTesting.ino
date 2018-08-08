#include <phys253.h>
#include <LiquidCrystal.h>

int motorPin = 0;
int knobVal = 0;


void setup() {
  // put your setup code here, to run once:
#include <phys253setup.txt>
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  knobVal = knob(6) - 512;
  motor.speed(motorPin,knobVal);
  LCD.clear(); LCD.home();
  LCD.print( knobVal);
  delay(50);
}
