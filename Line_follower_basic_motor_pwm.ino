#include <phys253.h>
#include <LiquidCrystal.h>

int LeftDark = 15;
int RightDark = 15;
int motorLeft = 3;
int motorRight = 2;

void setup(){
  #include <phys253setup.txt>
}


void loop() {

  while(!(startbutton())){
  LCD.clear(); LCD.home();
  LCD.print("Press Start");
  delay(100);
  }

  while(!(stopbutton())){
    LCD.clear(); LCD.home();

    

    int SensorLeft = analogRead(3);
    int SensorRight = analogRead(2);
    LCD.setCursor(0,1); LCD.print(SensorLeft + "||" + SensorRight);
    LCD.setCursor(0,0); 

    if(SensorLeft > LeftDark && SensorRight < RightDark){
      LCD.print("Turning Left...");
      motor.speed(motorLeft,700);
      motor.speed(motorRight, 100);
    }
    else{
      if(SensorLeft > LeftDark && SensorRight > RightDark){
        LCD.print("Driving Straight...");
        motor.speed(motorLeft, 700);
        motor.speed(motorRight, 500);
      }
      else{
        if(SensorLeft < LeftDark && SensorRight > RightDark){
          LCD.print("Turning Right...");
          motor.speed(motorLeft, 100);
          motor.speed(motorRight, 700);
        }
        else{
          LCD.print("Off Track...");
          motor.speed(motorLeft,700);
          motor.speed(motorRight, 0);
        }
      }
    }

    if(stopbutton()){
      break;
    }
    delay(50);
  }

  LCD.clear(); LCD.print("Out of Loop");

  motor.stop_all();
  delay(100);

}
