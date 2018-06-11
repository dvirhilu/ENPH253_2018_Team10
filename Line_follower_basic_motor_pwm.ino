#include <phys253.h>
#include <LiquidCrystal.h>

int leftDark = 15;
int rightDark = 15;
int motorLeft = 3;
int motorRight = 2;
int k_p = 0;
int k_i = 0;
int k_d = 0;
int current_time = 0;
int prev_time = 0;
double error = 0;
double prev_error = 0;  
double pid;
int default_speed = 700;
int integral = 0;
int derivative = 0;

void setup(){
  #include <phys253setup.txt>
}


void loop() {

  while(!(startbutton())){
  LCD.clear(); LCD.home();
  LCD.print("Press Start");
  delay(100);
  }

  prev_time = micros();
  while(!(stopbutton())){
    LCD.clear(); LCD.home();


    int sensorLeft = analogRead(3);
    int sensorRight = analogRead(2);
    LCD.setCursor(0,1); LCD.print(sensorLeft + "||" + sensorRight);
    LCD.setCursor(0,0); 

    current_time = micros();
    error = (sensorRight - rightDark) + (leftDark - sensorLeft);
    integral = prev_error + error*(current_time - prev_time);
    derivative = (error - prev_error)/(current_time - prev_time);

    pid = k_p*error + k_i*integral + k_d*derivative;

    if( pid < 0 ){
      motor.speed(motorLeft,default_speed);
      motor.speed(motorRight,default_speed - pid);
      LCD.print("------>>");
    }
    else{
      motor.speed(motorLeft,default_speed - pid);
      motor.speed(motorRight,default_speed);
      LCD.print("<<------");
    }

    prev_error = error;
    prev_time = current_time;
/*
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
    }*/

    if(stopbutton()){
      break;
    }
    delay(50);
  }

  LCD.clear(); LCD.print("Out of Loop");

  motor.stop_all();
  delay(100);
}
