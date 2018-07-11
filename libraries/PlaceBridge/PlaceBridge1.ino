#include <phys253.h>          
#include <LiquidCrystal.h>     

////////////////////////////////////////////////
// First Bridge Placing Test Program 
/////////////////////////////////////////////////
int motorLeftPin = 0;
int motorRightPin = 0;
int initialAngle = 0;
int finalAngle = 45;
int backupSpeed = -100;

void setup() {
  #include <phys253setup.txt>
  Serial.begin(9600) ;
  RCServo0.write(initialAngle);
}

void loop() {

  // back up at default speed for a short distance
    motor.speed(motorLeftPin, backupSpeed);
    motor.speed(motorRightPin, backupSpeed);
    delay(50);
    motor.stop_all();
    
    // drop the first bridge
    RCServo0.write(finalAngle);
    delay(1000);
    
    // back up for a short distance
    motor.speed(motorLeftPin, backupSpeed);
    motor.speed(motorRightPin, backupSpeed);
    delay(50);
    motor.stop_all();
    delay(500);
    
    // move forward and start tape following as soon as the tape is detected
    motor.speed(motorLeftPin, 100);
    motor.speed(motorRightPin, 100);
}
