#include <phys253.h>
#include <LiquidCrystal.h>
#include<MenuItem.h>
#include<avr/EEPROM.h>
//PID values
MenuItem kp = MenuItem("p_k_p", (unsigned int*)1);
MenuItem kd = MenuItem("p_k_d", (unsigned int*)2);
MenuItem gainz = MenuItem("p_gainzz", (unsigned int*)3);
MenuItem lDark = MenuItem("a_leftDark", (unsigned int*)4);
MenuItem rDark = MenuItem("a_rightDark", (unsigned int*)5);
MenuItem motor_speed = MenuItem("m_speeeeed", (unsigned int*)6);
MenuItem menu[] = {kp, kd, gainz, lDark, rDark, motor_speed};
int leftDark;
int rightDark;
int default_speed;
int gain;
int k_p;
int k_d;
int motorLeft = 2;
int motorRight = 3;
double current_time = 0;
double prev_time = 0;
double error = 0;
double prev_error = 0;
int derivative = 0;
double pid;
boolean leftError = true;
int a_knob_thresh = 100;
int s_knob_thresh = 270;
int p_knob_thresh = 100;
char analog_sensors = 'a';
char servos = 's';
char PID_constants = 'p';
void setup() {
#include <phys253setup.txt>
  Serial.begin(9600);
}
void loop() {
  while (!(startbutton())) {
    LCD.clear(); LCD.home();
    LCD.print("Press Start");
    delay(100);
  }
  delay(500);
  while (!(startbutton())) {
    menuToggle();
  }
  k_p = menu[0].getValue();
  k_d = menu[1].getValue();
  gain = menu[2].getValue();
  leftDark = menu[3].getValue();
  rightDark = menu[4].getValue();
  default_speed = menu[5].getValue();
  prev_time = micros();
  while (!(stopbutton()) && !(startbutton())) {
    int sensorLeft = analogRead(2);
    int sensorRight = analogRead(3);
    current_time = micros();
    LCD.clear();
    LCD.setCursor(0, 0);
    LCD.print((int)sensorLeft + String(" ") + (int)sensorRight + String(" ") + pid);
    LCD.setCursor(0, 1);
    //error = (sensorRight - rightDark) + (leftDark - sensorLeft);
    if (sensorRight < rightDark && !(sensorLeft < leftDark)) {
      leftError = false;
      error = 1;
    }
    else if ( !(sensorRight < rightDark) && sensorLeft < leftDark) {
      leftError = true;
      error = -1;
    }
    else if (sensorRight < rightDark && sensorLeft < leftDark) {
      if (leftError) {
        error = -5;
      }
      else {
        error = 5;
      }
    }
    else {
      error = 0;
    }
    derivative = 100000 * (error - prev_error) / (current_time - prev_time);
    pid = gain * (k_p * error + k_d * derivative);
    prev_error = error;
    prev_time = current_time;
    if ( pid < 0 ) {
      motor.speed(motorLeft, default_speed);
      motor.speed(motorRight, default_speed + pid);
      LCD.print(default_speed + String(" ") + (default_speed + pid));
    }
    else {
      motor.speed(motorLeft, default_speed - pid);
      motor.speed(motorRight, default_speed);
      LCD.print((default_speed - pid) + String(" ") + (default_speed));
    }
    delay(50);
  }
  LCD.clear(); LCD.print("reeeeeee");
  motor.stop_all();
  delay(100);
}
void menuToggle() {
  int sizeArray = sizeof(menu[0]);
  int value = knob(6);
  int menu_item = knob(7) * (sizeof(menu) / sizeArray) / 1024;
  if (menu_item > (sizeof(menu) / sizeArray) - 1) {
    menu_item = sizeof(menu) / sizeArray - 1;
  }
  else if (menu_item < 0) {
    menu_item = 0;
  }
  String itemName = (String)menu[menu_item].getName();
  char firstChar = itemName.charAt(0);
  LCD.clear(); LCD.home();
  LCD.print( itemName + " " );
  LCD.print( (String)menu[menu_item].getValue() + " " );
  LCD.setCursor(0, 1);
  if (firstChar == analog_sensors) {
    LCD.print(a_knob_thresh * (value / 1024.0));
    if (stopbutton()) menu[menu_item].setValue(a_knob_thresh * (value / 1024.0));
  }
  else if (firstChar == servos) {
    LCD.print(s_knob_thresh * (value / 1024.0));
    if (stopbutton()) menu[menu_item].setValue(s_knob_thresh * (value / 1024.0));
  }
  else if (firstChar == PID_constants) {
    LCD.print(p_knob_thresh * (value / 1024.0));
    if (stopbutton()) menu[menu_item].setValue(p_knob_thresh * (value / 1024.0));
  }
  else {
    LCD.print(value);
    if (stopbutton()) menu[menu_item].setValue(value);
  }
  delay(50);
}
