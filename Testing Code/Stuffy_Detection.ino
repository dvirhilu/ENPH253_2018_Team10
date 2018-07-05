#include <phys253.h>
#include <LiquidCrystal.h>

int pulse = 36;
int zero = 37;
int start_time;
int current_time = 0;

void setup() {
#include <phys253setup.txt>
  Serial.begin(9600);j 
  pinMode(pulse, OUTPUT);
}

void loop() {
  while (!(startbutton())) {
    LCD.clear(); LCD.home();
    LCD.print("Press Start");
    Serial.println("Press Start");
    delay(100);
  }
  delay(500);

  while (!startbutton()) {
    LCD.clear();
    start_time = millis();
    current_time = millis();

    analogWrite(zero,0);

        while (current_time < 100 + start_time) {
          analogWrite(pulse, 255);
          current_time = millis();
          Serial.println("high");
        }
        while (current_time < 200 + start_time) {
          analogWrite(pulse, 0);
          current_time = millis();
          Serial.println("low");
        }

  }
}
