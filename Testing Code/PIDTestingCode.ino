#include <phys253.h>
#include <LiquidCrystal.h>
#include<MenuItem.h>
#include<avr/EEPROM.h>\
//PID values

constexpr int motorLeft = 1;
constexpr int motorRight = 0;
constexpr int sensorLeftPin = 2;
constexpr int sensorRightPin = 3;
constexpr int a_knob_thresh = 100;
constexpr int s_knob_thresh = 270;
constexpr int p_knob_thresh = 200;
constexpr int stuffyComPin = 11;

MenuItem kp = MenuItem("p_k_p", (unsigned int*)1);
MenuItem kd = MenuItem("p_k_d", (unsigned int*)5);
MenuItem gainz = MenuItem("p_gainzz", (unsigned int*)9);
MenuItem lDark = MenuItem("a_leftDark", (unsigned int*)13);
MenuItem rDark = MenuItem("a_rightDark", (unsigned int*)17);
MenuItem motor_speed = MenuItem("m_speeeeed", (unsigned int*)21);
MenuItem menu[] = {kp, kd, gainz, lDark, rDark, motor_speed};
int leftDark;
int rightDark;
int default_speed;
int gain;
int k_p;
int k_d;
double current_time = 0;
double prev_time = 0;
double error = 0;
double prev_error = 0;
int derivative = 0;
double pid;
boolean leftError = true;
char analog_sensors = 'a';
char servos = 's';
char PID_constants = 'p';

void setup() {
#include <phys253setup.txt>
  Serial.begin(9600);
}

void loop() {
  while (!startbutton()) {
    menuToggle();
  }

  delay(500);
  k_p = menu[0].getValue();
  k_d = menu[1].getValue();
  gain = menu[2].getValue();
  leftDark = menu[3].getValue();
  rightDark = menu[4].getValue();
  default_speed = menu[5].getValue();
  prev_time = micros();

  LCD.print("analog ree");
  delay(1000);
  while (!(stopbutton()) && !(startbutton())) {
    if ( digitalRead(stuffyComPin) ) {
      motor.stop_all();
      while ( digitalRead(stuffyComPin) ) {}
    }
    int sensorLeft = analogRead(sensorLeftPin);
    int sensorRight = analogRead(sensorRightPin);
    current_time = micros();
    LCD.clear();
    LCD.setCursor(0, 0);
    LCD.print((int)sensorLeft + String(" ") + (int)sensorRight + String(" ") + error + String(" ") + derivative);

    LCD.setCursor(0, 1);

    if (sensorRight < rightDark && !(sensorLeft < leftDark)) {
      leftError = false;
      error =  rightDark - sensorRight;
    }
    else if ( !(sensorRight < rightDark) && sensorLeft < leftDark) {
      leftError = true;
      error = sensorLeft - leftDark;
    }
    else if (sensorRight < rightDark && sensorLeft < leftDark) {
      if (leftError) {
        error = (sensorRight - rightDark) + (sensorLeft - leftDark) ;
      }
      else {
        error = (rightDark - sensorRight) + (leftDark - sensorLeft);
      }
    }
    else {
      error = 0;
    }

    derivative = 100000 * (error - prev_error) / (current_time - prev_time);

    pid = gain * (k_p / 10.0 * error + k_d / 10.0 * derivative);

    prev_error = error;
    prev_time = current_time;

    if ( pid < 0 ) {
      motor.speed(motorLeft, default_speed - 0.5 * pid);
      motor.speed(motorRight, default_speed + 0.5 * pid);
      LCD.print((default_speed - pid) + String(" ") + default_speed);

    }
    else {
      motor.speed(motorLeft, default_speed - 0.5 * pid);
      motor.speed(motorRight, default_speed + 0.5 *
                  pid);
      LCD.print((default_speed) + String(" ") + (default_speed + pid));
    }
    delay(50);
  }

  LCD.clear(); LCD.print("reeeeeee");
  delay(500);
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
