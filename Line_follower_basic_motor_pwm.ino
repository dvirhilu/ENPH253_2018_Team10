#include <phys253.h>
#include <LiquidCrystal.h>

int leftDark = 15;
int rightDark = 15;
int motorLeft = 3;
int motorRight = 2;
double k_p = 0;
double k_i = 0;
double k_d = 0;
int current_time = 0;
int prev_time = 0;
double error = 0;
double prev_error = 0;  
double pid;
int default_speed = 700;
int integral = 0;
int derivative = 0;
boolean p_disp = false;
boolean i_disp = false;
boolean d_disp = true;
boolean leftError = true;

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

    double choosePID = knob(6);
    double valuePID = knob(7);

    if(startbutton()){
      if(d_disp){
        d_disp = false;
        p_disp = true;
      }
      else if(p_disp){
        p_disp = false;
        i_disp = true;
      }
      else{
        i_disp = false;
        d_disp = true;
      }
    }
    
    LCD.clear(); LCD.home();

    if(p_disp){
      k_p = valuePID/8;
      LCD.setCursor(0,1);
      LCD.print("K_p:" + k_p);
    }
    else if(i_disp){
      k_i = valuePID/200;
      LCD.setCursor(0,1);
      LCD.print("K_i:" + k_i);
    }
    else{
      k_d = valuePID/200;
      LCD.setCursor(0,1);
      LCD.print("K_d:" + k_d);
    }

    int sensorLeft = analogRead(3);
    int sensorRight = analogRead(2);
    LCD.setCursor(0,0); 

    current_time = micros();
    error = (sensorRight - rightDark) + (leftDark - sensorLeft);
    if ((sensorRight-rightDark)>5 && (sensorLeft-leftDark)<5){
      leftError = false;
    }
    if ((sensorRight-rightDark)<5 && (sensorLeft-leftDark)>5){
      leftError = true;
    }
    if ((sensorRight-rightDark)>5 && (sensorLeft-leftDark)>5){
      if(leftError){
        error -=5;
      }
      else{
        error +=5;
      }
    }
    integral = prev_error + error*(current_time - prev_time);
    derivative = (error - prev_error)/(current_time - prev_time);

    pid = k_p*error + k_i*integral + k_d*derivative;

    if( pid < 0 ){
      motor.speed(motorLeft,default_speed);
      motor.speed(motorRight,default_speed + pid);
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
