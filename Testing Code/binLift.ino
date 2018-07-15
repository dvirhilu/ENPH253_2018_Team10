// Bin Lifting Test Code 


#include <phys253.h>
#include <LiquidCrystal.h>
#include<MenuItem.h>
#include<avr/EEPROM.h>

//Pinout
constexpr int motorLeft = 1;
constexpr int motorRight = 0;
constexpr int motorlift = 2;
constexpr int sensorLeftPin = 0;
constexpr int sensorRightPin = 1;
constexpr int onekHzPin = 2;
constexpr int tenkHzPin = 3;
constexpr int stuffyComPin = 11;

//knob threshold & setup
constexpr int a_knob_thresh = 200;
constexpr int s_knob_thresh = 270;
constexpr int p_knob_thresh = 200;
char analog_sensors = 'a';
char servos = 's';
char PID_constants = 'p';

MenuItem kp = MenuItem("p_k_p", (unsigned int*)1);
MenuItem kd = MenuItem("p_k_d", (unsigned int*)5);
MenuItem gainz = MenuItem("p_gainzz", (unsigned int*)9);
MenuItem lDark = MenuItem("a_leftDark", (unsigned int*)13);
MenuItem rDark = MenuItem("a_rightDark", (unsigned int*)17);
MenuItem motor_speed = MenuItem("m_speeeeed", (unsigned int*)21);
MenuItem onekHz = MenuItem("a_1kHz", (unsigned int*)25);
MenuItem tenkHz = MenuItem("a_10kHz", (unsigned int*)29);
MenuItem menu[] = {kp, kd, gainz, lDark, rDark, motor_speed, onekHz, tenkHz};

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
int onekHzThresh;
int tenkHzThresh;
int onekHzReading = 0;
int tenkHzReading = 0;
int stuffyCount = 0;
bool archPassed = false;

int overheadCount = 0;
int duration = 0; 
double distance_sonar_arch = 0.3; // in meters

double sonarThreshold = (distance_sonar_arch * 2 / 343) * 1000000; 
int liftingSpeed = 100; // the speed to lift the bins

// in microseconds - the time for the sonar to detect the signal if an object is 30 cm above 
// if the sonar echo still did not detect the HIGH signal after this amount of time, there is nothing above. 

volatile unsigned int trigPin = 5; // digital pin #5 on TINAH for Sonar Trig Output Pin
volatile unsigned int echoPin = 4; // digital pin #4 on TINAH for Sonar Echo Input Pin
 

void setup() {
  #include <phys253setup.txt>
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  // trigger external interrupt when the sonar detects the archway above (signaled by a rise in voltage)
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
  onekHzThresh = menu[6].getValue();
  tenkHzThresh = menu[7].getValue();
  prev_time = micros();

  delay(1000);
  while (!(stopbutton()) && !(startbutton())) {
    /*
      if ( digitalRead(stuffyComPin) ) {
      motor.stop_all();
      stuffyCount++;
      while ( digitalRead(stuffyComPin) ) {}
      }
    */
    while (stuffyCount == 2 && archPassed == false) {
      if (analogRead(tenkHzThresh) > tenkHzThresh) {
        motor.stop_all();
        while (analogRead(onekHzPin) < onekHzThresh) {}
        while (analogRead(tenkHzPin) < tenkHzThresh) {}
        archPassed == true;
        break;
      }
      else if (analogRead(onekHzPin) > onekHzThresh) {
        motor.stop_all();
        while (analogRead(tenkHzPin) < tenkHzThresh) {}
        archPassed == true;
        break;
      }
      PID();
    }
    PID();

    // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    // Read the signal from the sensor: a HIGH pulse whose
    // duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    duration = pulseIn(echoPin, HIGH, sonarThreshold);

    // passing the archway 
    while (duration != 0 && overheadCount == 0) {
      PID();
      // passed the archway 
      if(duration == 0) {
        overheadCount = 1;
        delay(300); // let the whole robot pass the arch
        motor.stop_all();
        binLift1();
        int start_time = millis();
        while(/*1st limit switch is pressed (which indicate the 1st bin is not hooked to zipline yet)*/){
          PID();
          current_time = millis();
        }
        
        binLower();
        break;
      }
    }
  }

  LCD.clear(); LCD.print("reeeeeee");
  delay(500);
  motor.stop_all();
  delay(100);
}

// the program to lift the 1st bin after the archway is detected
void binLift1() {
  while(/*rotary encoder has not yet reached the bin 1 threshold height*/) {
    motorSpeed(motorLift, liftingSpeed);
    delay(250);
  }
  motorSpeed(motorLift, 0);
}

// the program to lift the 2nd bin after the archway is detected
void binLift2() {
  while(/*rotary encoder has not yet reached the bin 2 threshold height*/) {
    motorSpeed(motorLift, liftingSpeed);
    delay(250);
  }
  motorSpeed(motorLift, 0);
}

// the program to lower the bin after the bin is hooked to the zipline
void binLower() {
  while(/*rotary encoder has not yet reached the bin minimum threshold height*/) {
    motorSpeed(motorLift, -liftingSpeed);
    PID();
  }
  motorSpeed(motorLift, 0);
}


void PID() {
  int sensorLeft = analogRead(sensorLeftPin);
  int sensorRight = analogRead(sensorRightPin);
  current_time = micros() / 1000000.0;;
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
