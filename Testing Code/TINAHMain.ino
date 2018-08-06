#include <phys253.h>
#include <LiquidCrystal.h>
#include<MenuItem.h>
#include<avr/EEPROM.h>
#include <PID.h>
//PID values

constexpr int motorLeft = 2;
constexpr int motorRight = 0;
constexpr int motorLift = 1;
constexpr int sensorLeftPin = 0;
constexpr int sensorRightPin = 1;
constexpr int a_knob_thresh = 100;
constexpr int s_knob_thresh = 270;
constexpr int p_knob_thresh = 200;
constexpr int stuffyComPin = 6;
constexpr int comPin = 13; // change this
constexpr int encoderLeftPin = 1;
constexpr int encoderRightPin = 0;
constexpr int cpr = 20;
constexpr int gearRatio = 3;

volatile long encoderLPos = 0;  // a counter for the dial
volatile long encoderRPos = 0;  // a counter for the dial
volatile long encoderLiftPos = 0;

PID pid( sensorLeftPin, sensorRightPin, motorLeft, motorRight );

MenuItem kp = MenuItem("p_k_p", (unsigned int*)1);
MenuItem kd = MenuItem("p_k_d", (unsigned int*)5);
MenuItem gainz = MenuItem("p_gainzz", (unsigned int*)9);
MenuItem lDark = MenuItem("a_leftDark", (unsigned int*)13);
MenuItem rDark = MenuItem("a_rightDark", (unsigned int*)17);
MenuItem motor_speed = MenuItem("m_speeeeed", (unsigned int*)21);
MenuItem percent = MenuItem("p_percentage", (unsigned int*)25);
MenuItem edgeThresh = MenuItem("edgeThresh", (unsigned int*)29);
MenuItem stuffy_delay = MenuItem("stuffyDelay", (unsigned int*)33);
MenuItem archdel = MenuItem("p_archdel", (unsigned int*) 37);
MenuItem foroff = MenuItem("a_forOffset", (unsigned int*) 41);
MenuItem backupdel = MenuItem("a_backupDelay", (unsigned int*) 45);
MenuItem forwarddel = MenuItem("forwardDel", (unsigned int*) 49);
MenuItem stuffy3del = MenuItem("p_stuffy3del", (unsigned int*) 53);
MenuItem bridgeback = MenuItem( "a_bridgeback", (unsigned int*) 57);
MenuItem turndel = MenuItem( "turndel", (unsigned int*) 61);
MenuItem sharpTurnDel = MenuItem( "p_90degree", (unsigned int*) 65);
MenuItem backup3 = MenuItem("p_back3", (unsigned int*) 69);
MenuItem bridge2For = MenuItem("p_bridge2for", (unsigned int*) 73);
MenuItem dropoffdel = MenuItem("p_dropdel", (unsigned int*) 77);
MenuItem upSpeed = MenuItem("s_upSpeed", (unsigned int*) 81);
MenuItem downSpeed = MenuItem("s_downSpeed", (unsigned int*) 85);
MenuItem lift1 = MenuItem("p_lift1" , (unsigned int*) 89);
MenuItem lower1 = MenuItem("p_lower1", (unsigned int*) 93);
MenuItem irDelay = MenuItem("p_IRDelay", (unsigned int*) 97);

MenuItem menu[] = {kp, kd, gainz, lDark, rDark, motor_speed, percent, edgeThresh, stuffy_delay, archdel, foroff, backupdel, forwarddel, stuffy3del, bridgeback, turndel, sharpTurnDel, backup3, bridge2For, dropoffdel, upSpeed, downSpeed, lift1, lower1, irDelay};

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

// encoder interrupt stuff

ISR(INT0_vect) {
  encodeR();
}
ISR(INT1_vect) {
  encodeL();
}

ISR(INT2_vect) {
  encodeLift();
}

void enableExternalInterrupt(unsigned int INTX, unsigned int mode) {
  if (INTX > 3 || mode > 3 || mode == 1) return;
  cli();
  // Allow pin to trigger interrupts
  EIMSK |= (1 << INTX);
  // Clear the interrupt configuration bits
  EICRA &= ~(1 << (INTX * 2 + 0));
  EICRA &= ~(1 << (INTX * 2 + 1));
  // Set new interrupt configuration bits
  EICRA |= mode << (INTX * 2);
  sei();
}

void encodeL() {
  encoderLPos++;
}

void encodeR() {
  encoderRPos++;
}

void encodeLift() {
  encoderLiftPos++;
}

void setup() {
#include <phys253setup.txt>
  Serial.begin(9600);
  enableExternalInterrupt(INT0, FALLING);
  enableExternalInterrupt(INT1, FALLING);
  enableExternalInterrupt(INT2, FALLING);

  pinMode(encoderLeftPin, INPUT);
  pinMode(encoderRightPin, INPUT);

}

void loop() {

  while ( !startbutton() ) {
    initialScreen();
  }
  highCount = 0;
  edgeCount = 0;
  digitalWrite(comPin, LOW);
  LCD.print("we out here");
  delay(500);

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
  delay(4000);


  while (!stopbutton() && !startbutton()) {
    pid.tapeFollow();

    if ( debounceStuffyPin() && highCount != 3 ) {
      Serial.println(highCount + String("why are you here?"));
      arduinoStop();
      //digitalWrite(comPin,HIGH);
      highCount++;
      if ( highCount == 2) {
        digitalWrite(comPin, LOW);
        LCD.clear(); LCD.home();
        LCD.print("low");
      }
    }
    else if ( debounceStuffyPin() && highCount == 3) {
      Serial.println("go back you went too far");
      tapeFollowNTicks( archdel.getValue() );
      motor.stop_all();
      doYouEvenLiftBro();
      tapeFollowNTicks( dropoffdel.getValue());
      motor.stop_all();
      doYouEvenLowerBro();
      tapeFollowNTicks( stuffy3del.getValue());
      digitalWrite(comPin, HIGH);
      LCD.clear(); LCD.home();
      LCD.print("high");
      highCount++;
    }

    if (pid.isEdge()) {
      edgeCount++;
      if (edgeCount == 1) {
        placeBridge1();
        tapeFollowNTicks( irDelay.getValue() );
        digitalWrite(comPin, HIGH);
        LCD.clear(); LCD.home();
        LCD.print("high");
      }
      else if (edgeCount == 2) {
        toSecondGap();
        placeBridge2();
      }
    }
  }
  Serial.println("lol I died");

  LCD.clear(); LCD.print("reeeeeee");
  delay(2500);
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
  int forwardOffset = foroff.getValue();

  // For servo mounted on the left side of the robot
  int finalAngle = 120;

  // For servo mounted on the right side of the robot
  // int finalAngle = 135;


  //back up for a short distance to drop the bridge
  motor.speed(motorLeft, backupSpeed);
  motor.speed(motorRight, backupSpeed);

  delay(50);
  motor.stop_all();
  delay(250);

  // drop the first bridge
  //analogWrite(bridgeServoPin, finalAngle);

  // back up a little bit to drop the bridge
  motor.speed(motorLeft, backupSpeed);
  motor.speed(motorRight, backupSpeed);
  delayNTicks(bridgeback.getValue());
  motor.stop_all();
  RCServo0.write(finalAngle);
  RCServo0.write(finalAngle);
  delay(1500);

  // back up for a short distance to completely lay down the bridge
  motor.speed(motorLeft, backupSpeed);
  motor.speed(motorRight, backupSpeed);
  delayNTicks(backupdel.getValue());

  // move forward and start tape following as soon as the tape is detected
  motor.speed(motorLeft, -backupSpeed + forwardOffset);
  motor.speed(motorRight, -backupSpeed);
  delay(forwarddel.getValue() * 2.5);
  motor.speed(motorLeft, -backupSpeed - 100);
  motor.speed(motorRight, -backupSpeed + 50);
  delay(turndel.getValue() * 2);
}

void arduinoStop() {
  tapeFollowNTicks( stuffy_delay.getValue() );
  motor.stop_all();

  while ( digitalRead(stuffyComPin) == HIGH ) {}
}

void delayNTicks( int numTicks ) {
  long startTicks = encoderRPos;
  while ( encoderRPos - startTicks < numTicks ) {}
}

void liftNTicks( int numTicks ) {
  long startTicks = encoderLiftPos;
  while ( encoderLiftPos - startTicks < numTicks ) {

    LCD.clear(); LCD.home();
    LCD.print(encoderLiftPos);
    delay(10);
  }
}

void turn90Degrees() {
  motor.speed(motorLeft, 120);
  motor.speed(motorRight, -120);
  delayNTicks(sharpTurnDel.getValue());
}

void toSecondGap() {

  motor.stop_all();
  delay(200);

  motor.speed(motorRight, -120);
  motor.speed(motorLeft, -120);
  delayNTicks(backup3.getValue());

  turn90Degrees();

  while (!pid.isEdge()) {
    motor.speed(motorRight, 120);
    motor.speed(motorLeft, 120);
  }
}

void placeBridge2() {
  motor.speed(motorLeft, -120);
  motor.speed(motorRight, -120);
  delayNTicks(bridgeback.getValue());
  motor.stop_all();
  RCServo0.write(0);
  RCServo0.write(0);
  delay(1000);

  // back up for a short distance to completely lay down the bridge
  motor.speed(motorLeft, -120);
  motor.speed(motorRight, -120);
  delayNTicks(backupdel.getValue());

  motor.speed(motorLeft, 120);
  motor.speed(motorRight, 120);
  delayNTicks(bridge2For.getValue());
}

void tapeFollowNTicks( int numTicks ) {
  long startTicks = encoderRPos;
  while ( encoderRPos - startTicks < numTicks ) {
    pid.tapeFollow();
  }
}


bool debounceStartButton() {
  static bool prevState = false;
  static long debounceTime = 0;

  bool buttonState = startbutton();
  long currentTime = millis();

  if ( buttonState != prevState) {
    debounceTime = currentTime;
  }
  prevState = buttonState;

  return buttonState && (currentTime - debounceTime > 10);
}

bool debounceStopButton() {
  static bool prevState = false;
  static long debounceTime = 0;

  bool buttonState = stopbutton();
  long currentTime = millis();

  if ( buttonState != prevState) {
    debounceTime = currentTime;
  }
  prevState = buttonState;

  return buttonState && (currentTime - debounceTime > 10);
}

bool debounceStuffyPin() {
  if ( digitalRead(stuffyComPin) == HIGH ) {
    delay(5);
    if (digitalRead(stuffyComPin) == HIGH ) {
      return true;
    }
  }

  return false;
}

void doYouEvenLiftBro() {
  motor.speed(motorLift, -upSpeed.getValue());
  liftNTicks(lift1.getValue());
  motor.speed(motorLift, 0);
}

void doYouEvenLowerBro() {
  motor.speed(motorLift, downSpeed.getValue());
  liftNTicks(lower1.getValue());
  motor.speed(motorLift, 0);
}


