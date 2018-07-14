#include <phys253.h>
#include <LiquidCrystal.h>
#include<MenuItem.h>
#include<avr/EEPROM.h>\
//PID values

constexpr int motorLeft = 1;
constexpr int motorRight = 0;
constexpr int sensorLeftPin = 2;
constexpr int sensorRightPin = 3;
constexpr int a_knob_thresh = 200;
constexpr int s_knob_thresh = 270;
constexpr int p_knob_thresh = 200;
//constexpr int edge_thresh = 500;

MenuItem kp = MenuItem("p_k_p", (unsigned int*)1);
MenuItem kd = MenuItem("p_k_d", (unsigned int*)5);
MenuItem gainz = MenuItem("p_gainzz", (unsigned int*)9);
MenuItem lDark = MenuItem("a_leftDark", (unsigned int*)13);
MenuItem rDark = MenuItem("a_rightDark", (unsigned int*)17);
MenuItem motor_speed = MenuItem("m_speeeeed", (unsigned int*)21);
MenuItem edge = MenuItem("a_edge_thresh", (unsigned int*)25);
MenuItem menu[] = {kp, kd, gainz, lDark, rDark, motor_speed, edge};
int leftDark;
int rightDark;
int default_speed;
int gain;
int k_p;
int k_d;
int edge_thresh;
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
bool analog = false;
bool digital = false;

void setup() {
#include <phys253setup.txt>
  Serial.begin(9600);
  
}

void loop() {
  while (true) {
    LCD.clear(); LCD.home();
    LCD.print("Start for dig");
    LCD.setCursor(0, 1);
    LCD.print("Stop for analog");
    delay(100);
    if ( stopbutton()) {
      analog = true;
      Serial.println("analog");
      break;
    }
    if ( startbutton()) {
      digital = true;
      Serial.println("digital");
      break;
    }
  }
  delay(500);

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
  edge_thresh = menu[6].getValue();
  prev_time = micros();

  //if ( analog == true && digital == false) {
  //if (true) {
  LCD.print("analog ree");
  delay(1000);
  while (!(stopbutton()) && !(startbutton())) {
    int sensorLeft = analogRead(sensorLeftPin);
    int sensorRight = analogRead(sensorRightPin);
    current_time = micros();
    LCD.clear();
    LCD.setCursor(0, 0);
    LCD.print((int)sensorLeft + String(" ") + (int)sensorRight + String(" ") + error + String(" ") + derivative);

    LCD.setCursor(0, 1);


    //error = (sensorRight - rightDark) + (leftDark - sensorLeft);
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

    if((sensorRight > edge_thresh) || (sensorLeft > edge_thresh)) {
      placeBridge();
    }
    
  }
  //}
  /*else {
    LCD.print("digital ree");
    delay(1000);
    while (!(stopbutton()) && !(startbutton())) {
      int sensorLeft = analogRead(sensorLeftPin);
      int sensorRight = analogRead(sensorRightPin);
      current_time = micros();
      LCD.clear();
      LCD.setCursor(0, 0);
      LCD.print(error + String( " " ) + derivative + String(" ") + pid);
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
        motor.speed(motorLeft, default_speed - 0.5*pid);
        motor.speed(motorRight, default_speed + 0.5*pid);
        LCD.print((default_speed - pid) + String(" ") + default_speed);
      }
      else {
        motor.speed(motorLeft, default_speed - 0.5*pid);
        motor.speed(motorRight, default_speed + 0.5*pid);
        LCD.print((default_speed) + String(" ") + (default_speed + pid));
      }
      delay(50);
    }
    }*/
  LCD.clear(); LCD.print("reeeeeee");
  delay(500);
  motor.stop_all();
  delay(100);
}

////////////////////////////////////////////////
// First Bridge Placing Test Program 
/////////////////////////////////////////////////
void placeBridge() {
  int initialAngle = 45;
  int finalAngle = 0;
  int backupSpeed = -120;
  int forwardSpeed = 120;
  
  // back up for a short distance
  motor.speed(motorLeft, backupSpeed);
  motor.speed(motorRight, backupSpeed);
  delay(2000);
  motor.stop_all();
  delay(250);
  
  // drop the first bridge
  RCServo0.write(finalAngle);
  delay(600);

  // back up for a short distance to completely lay down the bridge
  motor.speed(motorLeft, backupSpeed);
  motor.speed(motorRight, backupSpeed);
  delay(300);
  motor.stop_all();

  // Resume the servo angle to initial position, not needed when the second bridge is added
  RCServo0.write(initialAngle);
  delay(300);

  // drive forward a short distance to get on to the black tape
  motor.speed(motorLeft, forwardSpeed);
  motor.speed(motorRight, forwardSpeed);
  delay(500);
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
