//based on PIDWithLibrary.ino 
//added bridge drop function 

#include <phys253.h>
#include <LiquidCrystal.h>
#include<MenuItem.h>
#include<avr/EEPROM.h>
#include <PID.h>
#include <Servo.h>
//#include <StuffyRescue.h>

//PID values
constexpr int motorLeft = 2;
constexpr int motorRight = 0;
constexpr int sensorLeftPin = 0;
constexpr int sensorRightPin = 3;
constexpr int a_knob_thresh = 100;
constexpr int s_knob_thresh = 270;
constexpr int p_knob_thresh = 200;

// sonar pins 
constexpr int trigPin = 8; // digital Output 
constexpr int echoPin = 0; //digital Input
bool archPassed = false;

// Bin Lift pins
constexpr int potPin = 0; // analog pin for potentiometer
constexpr int liftMotorPin = 3; // DC Motor pin #3 for GM7

// stuffy rescue
constexpr int stuffyComPin = 12;

int pulseLeft = 7;
int pulseRight = 8;
int sensorLeft = 0;
int sensorRight = 1;
int clawPinLeft = 9;
int armPinLeft = 10;
int clawPinRight = 5;
int armPinRight = 6;

// threshold values
int threshold = 150; // IR reading threshold
int clawAngle = 0; // servo starting angle (open position)
int armAngle = 150; // arm servo starting anlge (lifted position)
// initialize variables
int start_time = 0;
int current_time = 0;
int readingHighLeft = 0;
int readingLowLeft = 0;
int readingHighRight = 0;
int readingLowRight = 0;
Servo clawLeft;
Servo armLeft;
Servo clawRight;
Servo armRight;


constexpr int stuffyCount = 0;
/*
constexpr int IRThreshold = 0; // user change this 
constexpr int limitSwitchLeft; // add pin # 
constexpr int limitSwitchRight; // add pin # 
constexpr int armLeft; // add pin # 
constexpr int armRight; // add pin # 
constexpr int clawLeft; // add pin # 
constexpr int clawRight; // add pin # 
constexpr int sensorLeft; // add pin # 
constexpr int sensorRight; // add pin # 
constexpr int LEDLeft; // add pin # 
constexpr int LEDRight; // add pin # 
*/

// bridge 
constexpr int bridgeServoPin = 5;
constexpr int initialAngle = 0; // For servo mounted on the left side of the robot 
// int initialAngle = 180; // For servo mounted on the right side of the robot 
bool bridge1Placed = false;

// Instances
PID pid( sensorLeftPin, sensorRightPin, motorLeft, motorRight);
/*
StuffyRescue stuffyLeft( limitSwitchLeft, armLeft, clawLeft, sensorLeft, LEDLeft);
StuffyRescue stuffyRight(limitSwitchRight, armRight, clawRight, sensorRight, LEDRight);
*/

MenuItem kp = MenuItem("p_k_p", (unsigned int*)1);
MenuItem kd = MenuItem("p_k_d", (unsigned int*)5);
MenuItem gainz = MenuItem("p_gainzz", (unsigned int*)9);
MenuItem lDark = MenuItem("a_leftDark", (unsigned int*)13);
MenuItem rDark = MenuItem("a_rightDark", (unsigned int*)17);
MenuItem motor_speed = MenuItem("m_speeeeed", (unsigned int*)21);
MenuItem percent = MenuItem("percentage", (unsigned int*)25);
MenuItem edgeThresh = MenuItem("edgeThresh", (unsigned int*)29);
MenuItem menu[] = {kp, kd, gainz, lDark, rDark, motor_speed, percent, edgeThresh};
char analog_sensors = 'a';
char servos = 's';
char PID_constants = 'p';

void setup() {
  #include <phys253setup.txt>
  Serial.begin(9600);

  // Stuffy Rescue Set up 
  /*
  pinMode(pulseLeft, OUTPUT);
  pinMode(sensorLeft, INPUT);
  pinMode(pulseRight, OUTPUT);
  pinMode(sensorRight, INPUT);
  pinMode(stuffyComPin, OUTPUT);
  clawLeft.attach(clawPinLeft);
  armLeft.attach(armPinLeft);
  clawRight.attach(clawPinRight);
  armRight.attach(armPinRight);
 
  digitalWrite(stuffyComPin, LOW);
  clawLeft.write(clawAngle);
  armLeft.write(armAngle);
  clawRight.write(clawAngle);
  armRight.write(armAngle);
  delay(1000);
 
  // sonar set up 
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  */
  // bridge servo set up 
}

void loop() {
  RCServo0.write(0); 
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
  pid.setEdgeThresh( edgeThresh.getValue() );
  pid.initialize();

  LCD.clear(); LCD.home();
  LCD.print("REEEEEEEEEE");
  delay(1000);

  while (!(stopbutton()) && !(startbutton())) {
    pid.tapeFollow();
    //stuffyRescue();
    
    // 1st and 2nd Stuffy Detection (on the right side of the robot)
    //if(stuffyCount < 2) {
    //}
    
    // Edge Detection Check to place 1st bridge 
    if( pid.isEdge() ) {
      LCD.clear(); LCD.home();
      LCD.print( "edge detected" );
      delay(50);
      placeBridge1();
      bridge1Placed = true;
    }
    
    // IR Frequency Detection Stage
    //if(stuffyCount == 2 && archPassed == false) {
    //  frequencyDetection();
    //}

    // 1st Zipline Bin Lifting Stage 
    //if(archPassed == true) {
    //  zipline1();
    //}

    // Stuffy Rescue 
    
    
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
  int backupSpeed = -120;

  // For servo mounted on the left side of the robot 
  int finalAngle = 45;

  /* For servo mounted on the right side of the robot
     int finalAngle = 135;
   */

  // back up for a short distance to drop the bridge
  //motor.speed(motorLeft, backupSpeed);
  //motor.speed(motorRight, backupSpeed);

  //delay(400);
  LCD.print("stopping");
  motor.stop_all();
  delay(250);

  

  LCD.print("servo");
  // drop the first bridge
  //analogWrite(bridgeServoPin, finalAngle);
  RCServo0.write(finalAngle);
  delay(3000);

  // back up a little bit to drop the bridge
  motor.speed(motorLeft, backupSpeed);
  motor.speed(motorRight, backupSpeed);
  delay(1500);
  RCServo0.write(0);
  delay(1500);

  
  LCD.print("back it up");
  // back up for a short distance to completely lay down the bridge
  motor.speed(motorLeft, backupSpeed);
  motor.speed(motorRight, backupSpeed);
  delay(1000);

  // move forward and start tape following as soon as the tape is detected
  motor.speed(motorLeft, -backupSpeed);
  motor.speed(motorRight, -backupSpeed);
  delay(2000);
}

/*
 * Purpose: check the frequency of the IR beacon and respond accordingly
 */
void frequencyDetection() {
  int tenkHzPin;
  int tenkHzThresh;
  int onekHzPin;
  int onekHzThresh;
  
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




/*
 * Purpose: place the second bridge
 */
void placeBridge2() {
  int backupSpeed = -120;

  // For servo mounted on the left side of the robot 
  int finalAngle = 135;

  /* For servo mounted on the right side of the robot
     int finalAngle = 45;
   */


  // back up for a short distance to drop the bridge
  motor.speed(motorLeft, backupSpeed);
  motor.speed(motorRight, backupSpeed);

  delay(400);
  motor.stop_all();
  delay(250);
  
  // drop the second bridge
  analogWrite(bridgeServoPin, finalAngle);
  delay(1000);

  // back up for a short distance to completely lay down the bridge
  motor.speed(motorLeft, backupSpeed);
  motor.speed(motorRight, backupSpeed);
  delay(300);
  motor.stop_all();

  // move forward and start tape following as soon as the tape is detected
  motor.speed(motorLeft, -backupSpeed);
  motor.speed(motorRight, -backupSpeed);
  delay(1000);
}

/*
 * Purpose: Red the IR sensor and respond accordingly 
 */
void stuffyRescue() {
    start_time = millis();
    current_time = millis();
    digitalWrite(pulseLeft, HIGH);
    digitalWrite(pulseRight, HIGH);
    int average_reading_high_left = 0;
    int average_reading_high_right = 0;
    int average_reading_low_left = 0;
    int average_reading_low_right = 0;
    int count = 0;
    // 
    while (current_time < 50 + start_time) {
      current_time = millis();
      readingHighLeft = analogRead(sensorLeft);
      readingHighRight = analogRead(sensorRight);
      average_reading_high_left += readingHighLeft;
      average_reading_high_right += readingHighRight;
      count++;
    }

    average_reading_high_left /= count;
    average_reading_high_right /= count;
    count = 0;
    
    digitalWrite(pulseLeft, LOW);
    digitalWrite(pulseRight, LOW);
    
    while (current_time < 100 + start_time) {
      current_time = millis();
      readingLowLeft = analogRead(sensorLeft);
      readingLowRight = analogRead(sensorRight);
      average_reading_low_left += readingLowLeft;
      average_reading_low_right += readingLowRight;
      count++;
    }  

    average_reading_low_left /= count;
    average_reading_low_right /= count;
    count = 0;
    
    //Serial.println(readingHigh + String(" ") + readingLow + String(" ") + (readingHigh - readingLow));
    
    // Stuffy detected on left side 
    if (average_reading_high_left - average_reading_low_left > threshold) {
      delay(600); // add delay before stopping (sending HIGH to the TINAH)
      digitalWrite(stuffyComPin, HIGH);
      Serial.println("detected");
      armLeft.write(20); // change this after testing
      /*
        while (current_time < 400 + start_time) {
        current_time = millis();
        }
      */
      delay(2000);
      clawLeft.write(clawAngle + 28);
      delay(1000);
      /*
        while (true) {
        if (digitalRead(switchPin) == 0) {
          Serial.println("captured");
          break;
        }
        }
      */
      armLeft.write(200);
      /*while (current_time < 1000 + start_time) {
        current_time = millis();
        }*/
      delay(2000);
      clawLeft.write(clawAngle);
      delay(500);
      armLeft.write(armAngle);
      delay(1000);
      digitalWrite(stuffyComPin, LOW);
    }

    // Stuffy detected on right side 
    else if (average_reading_high_right - average_reading_low_right > threshold) {
      delay(600); // add delay before stopping (sending HIGH to the TINAH)
      digitalWrite(stuffyComPin, HIGH);
      Serial.println("detected");
      armRight.write(20); // change this after testing
      /*
        while (current_time < 400 + start_time) {
        current_time = millis();
        }
      */
      delay(2000);
      clawRight.write(clawAngle + 28);
      delay(1000);
      /*
        while (true) {
        if (digitalRead(switchPin) == 0) {
          Serial.println("captured");
          break;
        }
        }
      */
      armRight.write(200);
      /*while (current_time < 1000 + start_time) {
        current_time = millis();
        }*/
      delay(2000);
      clawRight.write(clawAngle);
      delay(500);
      armRight.write(armAngle);
      delay(1000);
      digitalWrite(stuffyComPin, LOW);
    }
}
