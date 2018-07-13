#include <phys253.h>          
#include <LiquidCrystal.h>     

////////////////////////////////////////////////
// First Bridge Placing Test Program 
/////////////////////////////////////////////////
int motorLeftPin = 1;
int motorRightPin = 0;
int initialAngle = 0;
int finalAngle = 45;
int backupSpeed = -120;

void setup() {
  #include <phys253setup.txt>
  Serial.begin(9600) ;
  RCServo0.write(initialAngle);
}

void loop() {

    LCD.clear();  LCD.home() ;
    LCD.setCursor(0,0); LCD.print("Bridge Drop Demo");
    LCD.setCursor(0,1); LCD.print("ENPH 253 TEAM 10");
    delay(500);
    while ( !(startbutton()) );  
    delay(3000);
    // back up at default speed for a short distance
    motor.speed(motorLeftPin, backupSpeed);
    motor.speed(motorRightPin, backupSpeed);
    delay(400);
    motor.stop_all();
    delay(250);
    // drop the first bridge
    RCServo0.write(finalAngle);
    delay(1000);
  
    // back up for a short distance
    motor.speed(motorLeftPin, backupSpeed);
    motor.speed(motorRightPin, backupSpeed);
    delay(300);
    motor.stop_all();
    RCServo0.write(initialAngle);
    delay(500);
    
    // move forward and start tape following as soon as the tape is detected
    motor.speed(motorLeftPin, 120);
    motor.speed(motorRightPin, 120);
    delay(1000);
    
}
