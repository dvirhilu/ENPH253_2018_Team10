#include <phys253.h>
#include <LiquidCrystal.h>
#include<MenuItem.h>
#include<avr/EEPROM.h>
#include <PID.h>
#include <Zipline.h>
//PID values

constexpr int motorLeft = 2;
constexpr int motorRight = 0;
constexpr int sensorLeftPin = 0;
constexpr int sensorRightPin = 3;
constexpr int a_knob_thresh = 100;
constexpr int s_knob_thresh = 270;
constexpr int p_knob_thresh = 200;
constexpr int stuffyComPin = 6;
constexpr int edgeComPin = 13; // change this later

// Zipline Lifting Mechanism
volatile unsigned int trigPin = 8; // digital pin #8 on TINAH for Sonar Trig Output Pin
volatile unsigned int echoPin = 7; // digital pin #7 on TINAH for Sonar Echo Input Pin
volatile unsigned int PotPin = 5; // analog pin #5 on TINAH for Potentiometer 
volatile unsigned int liftMotorPin = 1; // motor pin #1 for the lifting motor 
int start_time; 
int current_time;
// Zipline Initialization 
Zipline zipline(PotPin, echoPin, trigPin, liftMotorPin);

PID pid( sensorLeftPin, sensorRightPin, motorLeft, motorRight );

MenuItem kp = MenuItem("p_k_p", (unsigned int*)1);
MenuItem kd = MenuItem("p_k_d", (unsigned int*)5);
MenuItem gainz = MenuItem("p_gainzz", (unsigned int*)9);
MenuItem lDark = MenuItem("a_leftDark", (unsigned int*)13);
MenuItem rDark = MenuItem("a_rightDark", (unsigned int*)17);
MenuItem motor_speed = MenuItem("m_speeeeed", (unsigned int*)21);
MenuItem percent = MenuItem("p_percentage", (unsigned int*)25);
MenuItem edgeThresh = MenuItem("edgeThresh", (unsigned int*)29);
MenuItem stuffy_delay = MenuItem("_stuffyDelay", (unsigned int*)33);
MenuItem backoff = MenuItem("a_backupOffset", (unsigned int*) 37);
MenuItem foroff = MenuItem("a_forOffset", (unsigned int*) 41);
MenuItem backupdel = MenuItem("backupDelay", (unsigned int*) 45);
MenuItem forwarddel = MenuItem("forwardDel", (unsigned int*) 49);
MenuItem stuffy3del = MenuItem("stuffy3del", (unsigned int*) 53);
MenuItem bridgeback = MenuItem( "bridgeback", (unsigned int*) 57);
MenuItem turndel = MenuItem( "turndel", (unsigned int*) 61);
MenuItem menu[] = {kp, kd, gainz, lDark, rDark, motor_speed, percent, edgeThresh, stuffy_delay, backoff, foroff, backupdel, forwarddel, stuffy3del, bridgeback, turndel};

char analog_sensors = 'a';
char servos = 's';
char PID_constants = 'p';

int highCount = 0;
/* counts the number of times the Arduino send HIGH to the TINAH
    highCount 0 & 1 = first two stuffies
    highCount 2 = frequency detection
    highCount 3 = arch way
*/
int edgeCount = 0;
/* counts the number of edges the robot detects
    edgeCount 1 = first brdige
    edgeCount 2 = end of tape
    edgeCount 3 = second bridge
*/

void setup() {
  #include <phys253setup.txt>
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {

  while ( !startbutton() ) {
    initialScreen();
  }
  highCount = 0;
  edgeCount = 0;
  digitalWrite(edgeComPin, LOW);

  delay(500);
  RCServo0.write(180);
  RCServo0.write(180);

  while (!startbutton()) {
    menuToggle();
  }
  delay(500);

  pid.setKp( kp.getValue() );
  pid.setKd( kd.getValue() );
  pid.setGain( gainz.getValue() );
  pid.setLeftDark( lDark.getValue() );
  pid.setRightDark( rDark.getValue() );
  pid.setDefaultSpeed( motor_speed.getValue() );
  pid.setRatio( percent.getValue() );
  pid.setEdgeThresh( edgeThresh.getValue() );
  pid.initialize();

  LCD.clear(); LCD.home();
  LCD.print("REEEEEEEEEE");
  delay(1000);


  while (!(stopbutton()) && !(startbutton())) {
    pid.tapeFollow();

    if ( digitalRead(stuffyComPin) == HIGH && highCount != 3 ) {
      arduinoStop();
    }
    else if ( digitalRead(stuffyComPin) == HIGH && highCount == 3) {
      // let the whole robot pass the zipline 
      start_time = millis();
      current_time = millis() - start_time;
      while(current_time < 500){
          pid.tapeFollow();
          current_time = millis() - start_time;
      }

      // stop the motors and lift the bin 
      motor.stop_all();
      zipline.binLift1();

      // continue driving forward for a while till the zipline has passed 
      start_time = millis();
      current_time = millis() - start_time;
      // drive forward for 4000 milliseconds before lowering the bin
      while(current_time < 4000){
          pid.tapeFollow();
          current_time = millis() - start_time;
      }
      zipline.binLower();
    }
    
    if (pid.isEdge()) {
      edgeCount++;
      if (edgeCount == 1) {
        placeBridge1();
      }
      else if (edgeCount == 2) {
        digitalWrite(edgeComPin, HIGH);
        edgeBackup();
      }
      else if (edgeCount == 3) {
        // placeBridge2(); pseudo code for second bridge
        while ( !stopbutton() ) {
          motor.stop_all();
        }
      }
    }
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

void initialScreen() {
  LCD.clear(); LCD.home();
  LCD.print("Press Start");
  LCD.setCursor(0, 1);
  LCD.print("for REEEEEEEEE");
  delay(50);
}

/*
   Purpose: place the first bridge
*/
void placeBridge1() {
  int backupSpeed = -120;
  int backupOffset = backoff.getValue();
  int forwardOffset = foroff.getValue();

  // For servo mounted on the left side of the robot
  int finalAngle = 120;

  /* For servo mounted on the right side of the robot
     int finalAngle = 135;
  */

  //back up for a short distance to drop the bridge
  motor.speed(motorLeft, backupSpeed);
  motor.speed(motorRight, backupSpeed);

  delay(50);
  LCD.clear(); LCD.home();
  LCD.print("stopping");
  motor.stop_all();
  delay(250);


  LCD.clear(); LCD.home();
  LCD.print("servo");
  // drop the first bridge
  //analogWrite(bridgeServoPin, finalAngle);

  // back up a little bit to drop the bridge
  motor.speed(motorLeft, backupSpeed);
  motor.speed(motorRight, backupSpeed);
  delay(bridgeback.getValue());
  motor.stop_all();
  RCServo0.write(finalAngle);
  RCServo0.write(finalAngle);
  delay(1500);

  LCD.clear(); LCD.home();
  LCD.print("back it up");
  // back up for a short distance to completely lay down the bridge
  motor.speed(motorLeft, backupSpeed - backupOffset);
  motor.speed(motorRight, backupSpeed);
  delay(backupdel.getValue());

  // move forward and start tape following as soon as the tape is detected
  motor.speed(motorLeft, -backupSpeed + forwardOffset);
  motor.speed(motorRight, -backupSpeed);
  delay((int)(forwarddel.getValue() * 2.5));
  motor.speed(motorLeft, -backupSpeed - 100);
  motor.speed(motorRight, -backupSpeed + 50);
  delay(turndel.getValue() * 2);
}

void edgeBackup() {

  int backupSpeed = -100;
  int forwardDelay = stuffy3del.getValue();

  motor.stop_all();

  while ( digitalRead(stuffyComPin) == LOW ) {
    motor.speed( motorLeft, backupSpeed);
    motor.speed( motorRight, backupSpeed);
  }

  motor.stop_all();

  motor.speed( motorLeft, -backupSpeed);
  motor.speed( motorRight, -backupSpeed);
  delay( forwardDelay );
  motor.stop_all();
  while ( digitalRead(stuffyComPin) == HIGH) {}

}

void arduinoStop() {
  int stuffyDelay = stuffy_delay.getValue();
  delay(stuffyDelay);
  highCount++;
  motor.stop_all();
  while ( digitalRead(stuffyComPin) == HIGH ) {
    LCD.clear(); LCD.home();
    LCD.print("INSIDE");
    delay(50);
  }
  LCD.clear(); LCD.home();
  LCD.print("OUTSIDE");
  delay(50);
  LCD.clear(); LCD.home();
}
