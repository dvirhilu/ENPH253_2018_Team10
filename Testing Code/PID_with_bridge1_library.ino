//based on PIDWithLibrary.ino 
//added bridge drop function 

#include <phys253.h>
#include <LiquidCrystal.h>
#include<MenuItem.h>
#include<avr/EEPROM.h>
#include <PID.h>
//PID values

constexpr int motorLeft = 2;
constexpr int motorRight = 0;
constexpr int sensorLeftPin = 0;
constexpr int sensorRightPin = 3;
constexpr int a_knob_thresh = 100;
constexpr int s_knob_thresh = 270;
constexpr int p_knob_thresh = 200;
constexpr int stuffyComPin = 11;
// Bridge 
bool bridge1Placed = false;
int edgeThreshold = 300;

// sonar pins 
constexpr int trigPin = 8; // digital Output 
constexpr int echoPin = 0; //digital Input
bool archPassed = false;

// Bin Lift pins
constexpr int potPin = 0; // analog
constexpr int liftMotorPin = 3; // DC Motor pin #3 

// stuffy 
int stuffyCount = 0;

PID pid( sensorLeftPin, sensorRightPin, motorLeft, motorRight );

MenuItem kp = MenuItem("p_k_p", (unsigned int*)1);
MenuItem kd = MenuItem("p_k_d", (unsigned int*)5);
MenuItem gainz = MenuItem("p_gainzz", (unsigned int*)9);
MenuItem lDark = MenuItem("a_leftDark", (unsigned int*)13);
MenuItem rDark = MenuItem("a_rightDark", (unsigned int*)17);
MenuItem motor_speed = MenuItem("m_speeeeed", (unsigned int*)21);
MenuItem percent = MenuItem("percentage", (unsigned int*)25);
MenuItem menu[] = {kp, kd, gainz, lDark, rDark, motor_speed, percent};
char analog_sensors = 'a';
char servos = 's';
char PID_constants = 'p';

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
  delay(500);

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
  pid.initialize();

  LCD.clear(); LCD.home();
  LCD.print("REEEEEEEEEE");
  delay(1000);

  while (!(stopbutton()) && !(startbutton())) {
    pid.tapeFollow();

    // Edge Detection Check 
    if(bridge1Placed == false && analogRead(sensorLeftPin) > edgeThreshold) {
      placeBridge1();
      bridge1Placed = true;
    }

    // After Crossing Bridge 1, Left Turn Correction Stage
    if(bridge1Placed == true && (analogRead(sensorRightPin) < rDark.getValue())) {
      afterBridge1Correction();
    }

    // IR Frequency Detection Stage
    if(stuffyCount == 2 && archPassed == false) {
      FrequencyDetection();
    }

    // 1st Zipline Bin Lifting Stage 
    if(archPassed == true) {
      zipline1();
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
 * Purpose: place the first bridge 
 */
void placeBridge1() {
  int motorLeftPin = 1;
  int motorRightPin = 0;
  int bridgeServoPin = 3;
  int initialAngle = 0;
  int finalAngle = 45;
  int backupSpeed = -120;

  // back up for a short distance to drop the bridge
  motor.speed(motorLeftPin, backupSpeed);
  motor.speed(motorRightPin, backupSpeed);

  delay(400);
  motor.stop_all();
  delay(250);
  
  // drop the first bridge
  analogWrite(bridgeServoPin, finalAngle);
  delay(1000);

  // back up for a short distance to completely lay down the bridge
  motor.speed(motorLeftPin, backupSpeed);
  motor.speed(motorRightPin, backupSpeed);
  delay(300);
  motor.stop_all();

  // move forward and start tape following as soon as the tape is detected
  motor.speed(motorLeftPin, -backupSpeed);
  motor.speed(motorRightPin, -backupSpeed);
  delay(1000);
}

/*
 * Purpose: correct the orientation of the robot after crossing the first bridge to relocate the tape
 */
void afterBridge1Correction() {
  pid.tapeFollow();
}

/*
 * Purpose: check the frequency of the IR beacon and respond accordingly
 */
void FrequencyDetection() {
  while (stuffyCount == 2 && archPassed == false) {
    if (analogRead(tenkHzPin) > tenkHzThresh) {
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
    pid.tapeFollow();
  }
}

/*
 * Purpose: program that controls the movement of the robot between the archway and the first zipline
 */
void zipline1() {
  long duration; 
  int distance;
  double pot_reading; // reading from the potentiometer
  int angle; // angle position of the potentiometer
  double distance_sonar_arch = 35; // in cm
  int motorSpeed = 80;

  volatile unsigned int initialAngle = 90;
  volatile unsigned int maxAngle = 270;
  volatile unsigned int bin1Angle = initialAngle + 45;
  volatile unsigned int bin2Angle = initialAngle + 90;
  
  int start_time; 
  int current_time;
  int liftingSpeed = 100; // the speed to lift the bins

  digitalWrite(trigPin, LOW);

  for (int i = 0; i < 5; i++) {
      // Clears the trigPin
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
  
      // Sets the trigPin on HIGH state for 10 micro seconds
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);

      // Reads the echoPin, returns the sound wave travel time in microseconds
      duration = pulseIn(echoPin, HIGH);
      // Calculating the distance
      distance= duration*0.034/2;
  } 

  
  while(distance >= distance_sonar_arch) {
    pid.tapeFollow();

    // Clears the trigPin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    distance = duration*0.0343/2;

    // passed the archway 
    if(distance <= distance_sonar_arch) {
      // let the whole robot pass the arch
      delay(300); 
      motor.stop_all();

      // Lifting the bin
      pot_reading = analogRead(potPin);
      angle = (pot_reading / 1000) * maxAngle;
      
      while(angle < bin1Angle ) {
        motor.speed(liftMotorPin, motorSpeed);
        pot_reading = analogRead(potPin);
        angle = (pot_reading / 1000) * maxAngle;
      }
      motor.speed(liftMotorPin, 0);
      
      start_time = millis();
      current_time = millis() - start_time;
      
      // drive forward for 4000 milliseconds before lowering the bin
      while(current_time < 4000){
        pid.tapeFollow();
        current_time = millis() - start_time;
      }

      // Lowering the bin
      pot_reading = analogRead(potPin);
      angle = (pot_reading / 1000) * maxAngle;

      
      while(angle > initialAngle ) {
        motor.speed(liftMotorPin, -motorSpeed);
        pot_reading = analogRead(potPin);
        angle = (pot_reading / 1000) * maxAngle;
      }
      motor.speed(liftMotorPin, 0);
          break;
    }
  }
}

