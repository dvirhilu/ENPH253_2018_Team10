#include <phys253.h>
#include <LiquidCrystal.h>
#include<MenuItem.h>
#include<avr/EEPROM.h>
#include <PID.h>
#include <EdgePID.h>

constexpr int motorLeft = 2;
constexpr int motorRight = 0;
constexpr int motorLift = 1;
constexpr int sensorLeftPin = 0;
constexpr int sensorRightPin = 1;
constexpr int edgeLeft = 2;
constexpr int edgeRight = 3;
constexpr int a_knob_thresh = 100;
constexpr int s_knob_thresh = 270;
constexpr int p_knob_thresh = 200;
constexpr int stuffyComPin = 4;
constexpr int comPin = 13; // change this
constexpr int encoderLeftPin = 1;
constexpr int encoderRightPin = 0;


constexpr int cpr = 20;
constexpr int gearRatio = 3;

volatile long encoderLPos = 0;  // a counter for the dial
volatile long encoderRPos = 0;  // a counter for the dial
volatile long encoderLiftPos = 0;
int arduinoStopCount = 0;

PID pid( sensorLeftPin, sensorRightPin, motorLeft, motorRight );
EdgePID edgePID( edgeLeft, edgeRight, motorLeft, motorRight );

MenuItem kp = MenuItem("p_k_p", (unsigned int*)1);
MenuItem kd = MenuItem("p_k_d", (unsigned int*)5);
MenuItem stuffyDelayLeft = MenuItem("p_stuffdel_L", (unsigned int*)9);
MenuItem lDark = MenuItem("a_leftDark", (unsigned int*)13);
MenuItem rDark = MenuItem("a_rightDark", (unsigned int*)17);
MenuItem motor_speed = MenuItem("m_speeeeed", (unsigned int*)21);
MenuItem irspeed = MenuItem("IRSpeed", (unsigned int*)25);
MenuItem edgeThresh = MenuItem("edgeThresh", (unsigned int*)29);
MenuItem stuffy_delay = MenuItem("stuffyDelay", (unsigned int*)33);
MenuItem archdel = MenuItem("p_archdel", (unsigned int*) 37);
MenuItem foroff = MenuItem("a_forOffset", (unsigned int*) 41);
MenuItem backupdel = MenuItem("a_backupDelay", (unsigned int*) 45);
MenuItem forwarddel = MenuItem("p_forwardDel", (unsigned int*) 49);
MenuItem stuffy3del = MenuItem("p_stuffy3del", (unsigned int*) 53);
MenuItem bridgeback = MenuItem( "a_bridgeback", (unsigned int*) 57);
MenuItem turndel = MenuItem( "p_turndel", (unsigned int*) 61);
MenuItem sharpTurnDel = MenuItem( "p_90degree", (unsigned int*) 65);
MenuItem backup3 = MenuItem("p_back3", (unsigned int*) 69);
MenuItem bridge2For = MenuItem("p_bridge2for", (unsigned int*) 73);
MenuItem dropoffdel = MenuItem("p_dropdel", (unsigned int*) 77);
MenuItem upSpeed = MenuItem("s_upSpeed", (unsigned int*) 81);
MenuItem downSpeed = MenuItem("s_downSpeed", (unsigned int*) 85);
MenuItem lift1 = MenuItem("p_lift1" , (unsigned int*) 89);
MenuItem lower1 = MenuItem("p_lower1", (unsigned int*) 93);
MenuItem irDelay = MenuItem("p_IRDelay", (unsigned int*) 97);
MenuItem bridge2Back = MenuItem("a_bridge2bac", (unsigned int*) 101);
MenuItem backupdel2 = MenuItem("a_backupdel2", (unsigned int*) 105);
MenuItem edgeKp = MenuItem("a_edgeKp", (unsigned int*) 109);
MenuItem turn1Offset = MenuItem("a_turn1offset", (unsigned int*) 113);
MenuItem turn2Offset = MenuItem("a_turn2offset", (unsigned int*) 117);
MenuItem turn2del = MenuItem("p_turn2del", (unsigned int*) 121);
MenuItem susdelay = MenuItem("_susdelay", (unsigned int*) 125);
MenuItem lift2 = MenuItem("p_lift2", (unsigned int*) 129);
MenuItem lower2 = MenuItem("p_lower2", (unsigned int*) 133);
MenuItem dropdel2 = MenuItem("p_dropdel2", (unsigned int*) 137);
MenuItem backoff = MenuItem("p_backoff", (unsigned int*) 141);

MenuItem menu[] = {
  kp, kd, stuffyDelayLeft, lDark, rDark, motor_speed, irspeed, edgeThresh, stuffy_delay,
  archdel, foroff, backupdel, forwarddel, stuffy3del, bridgeback, turndel, sharpTurnDel,
  backup3, bridge2For, dropoffdel, upSpeed, downSpeed, lift1, lower1, irDelay, bridge2Back,
  backupdel2, edgeKp, turn1Offset, turn2Offset, turn2del, susdelay, lift2, lower2, dropdel2,
  backoff
};

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
int numBridgeDropped = 0;

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
  enableExternalInterrupt(INT0, FALLING);
  enableExternalInterrupt(INT1, FALLING);
  enableExternalInterrupt(INT2, FALLING);

  pinMode(encoderLeftPin, INPUT);
  pinMode(encoderRightPin, INPUT);
  pinMode(stuffyComPin, INPUT);
  pinMode(comPin, OUTPUT);

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
  pid.setGain( 1 );
  pid.setLeftDark( lDark.getValue() );
  pid.setRightDark( rDark.getValue() );
  pid.setDefaultSpeed( motor_speed.getValue() );
  pid.setRatio( 50 );
  pid.setEdgeThresh( edgeThresh.getValue() );
  pid.initialize();
  edgePID.setEdgeKp( edgeKp.getValue() );
  edgePID.setDefaultSpeed( motor_speed.getValue() );

  LCD.clear(); LCD.home();
  LCD.print("REEEEEEEEEE");


  while (!stopbutton() && !startbutton() ) {
    //LCD.print( highCount + String(" ") + digitalRead(stuffyComPin) + String("above"));
    //delay(50);
    if ( debounceStuffyPin() && highCount == 0 && numBridgeDropped == 0) {
      LCD.clear();
      LCD.print("stuffy 1");
      arduinoStop();
      while ( debounceLowStuffyPin() ) {}
      highCount++;
    }
    else if (debounceStuffyPin() && highCount == 1 && numBridgeDropped == 1) {
      LCD.clear();
      LCD.print("IR");
      arduinoStop();
      pid.setDefaultSpeed( irspeed.getValue() );
      while ( debounceLowStuffyPin() ) {}
      highCount++;
      digitalWrite(comPin, LOW);
    }
    else if ( debounceStuffyPin() && highCount == 2 && numBridgeDropped == 1) {
      LCD.clear();
      LCD.print("stuffy 2");
      //speed up after this through the arch
      arduinoStop();
      while ( debounceLowStuffyPin() ) {}
      highCount++;
    }
    else if ( pid.isEdge() && highCount == 1 && numBridgeDropped == 0) {
      LCD.clear(); LCD.home();
      LCD.print("edge");
      placeBridge1();
      tapeFollowNTicks( irDelay.getValue() );
      digitalWrite(comPin, HIGH);
      numBridgeDropped++;
    }
    else if ( debounceStuffyPin() && highCount == 3 && numBridgeDropped == 1) {
      LCD.clear(); LCD.home();
      LCD.print("arch");
      while ( debounceLowStuffyPin() ) {}
      highCount++;
    }
    else if ( debounceStuffyPin() && highCount == 4 && numBridgeDropped == 1) {
      LCD.clear(); LCD.home();
      LCD.print("storm trooper");
      pid.setDefaultSpeed( motor_speed.getValue() );
      motor.stop_all();
      doYouEvenLiftBro( lift1.getValue() );
      tapeFollowNTicks( dropoffdel.getValue());
      motor.stop_all();
      doYouEvenLowerBro( lower1.getValue() );
      tapeFollowNTicks( stuffy3del.getValue());
      digitalWrite(comPin, HIGH);
      while ( debounceLowStuffyPin() ) {}
      highCount++;
    }
    else if ( debounceStuffyPin() && highCount == 5 && numBridgeDropped == 1) {
      LCD.clear(); LCD.home();
      LCD.print("something Left");
      arduinoLeftStop();
      while ( debounceLowStuffyPin() ) {}
      highCount++;
    }
    else if ( pid.isEdge() && highCount > 5 ) {
      toSecondGap();
      placeBridge2();
      suspensionBridge();
      grandFinale();
      break;
    }
    //LCD.clear(); LCD.home();
    //LCD.print( highCount + String(" ") + digitalRead(stuffyComPin) + String("below") + numBridgeDropped);
    //delay(50);
    pid.tapeFollow();
  }


  /*
    while (!stopbutton() && !startbutton()) {
      pid.tapeFollow();

      if ( debounceStuffyPin() && highCount < 3 ) {
        Serial.println(highCount + String("why are you here?"));
        arduinoStop();
        //digitalWrite(comPin,HIGH);
        highCount++;
        if ( highCount == 2) {
          digitalWrite(comPin, LOW);
          LCD.clear(); LCD.home();
          LCD.print("low");
          pid.setDefaultSpeed( irspeed.getValue() );
        }
      }
      else if ( debounceStuffyPin() && highCount == 3) {
        Serial.println("go back you went too far");
        pid.setDefaultSpeed( motor_speed.getValue() );
        motor.stop_all();
        doYouEvenLiftBro( lift1.getValue() );
        tapeFollowNTicks( dropoffdel.getValue());
        motor.stop_all();
        doYouEvenLowerBro( lower1.getValue() );
        tapeFollowNTicks( stuffy3del.getValue());
        digitalWrite(comPin, HIGH);
        LCD.clear(); LCD.home();
        LCD.print("high");
        highCount++;
      }
      else if ( debounceStuffyPin() && highCount > 3) {
        arduinoLeftStop();
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
          suspensionBridge();
          grandFinale();
          break;
        }
      }
    }*/

  LCD.clear(); LCD.print("reeeeeee");
  delay(2500);
  motor.stop_all();
  delay(100);

}

void menuToggle() {
  int sizeArray = sizeof(menu[0]);
  int value = knob(6);
  int menu_item = knob(7) * (sizeof(menu) / sizeArray) / 998;
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
  motor.speed(motorLeft, backupSpeed - backoff.getValue());
  motor.speed(motorRight, backupSpeed);
  delayNTicks(backupdel.getValue());

  // move forward and start tape following as soon as the tape is detected
  motor.speed(motorLeft, -backupSpeed + forwardOffset);
  motor.speed(motorRight, -backupSpeed);
  delayNTicks(forwarddel.getValue());
  motor.speed(motorLeft, -backupSpeed - turn1Offset.getValue());
  motor.speed(motorRight, -backupSpeed );
  delayNTicks(turndel.getValue());
}

void arduinoStop() {
  LCD.setCursor(2, 0);
  LCD.print("arduinoStop");
  tapeFollowNTicks( stuffy_delay.getValue() );
  motor.stop_all();
}

void arduinoLeftStop() {
  arduinoStopCount++;
  LCD.clear(); LCD.home();
  LCD.print(highCount);
  tapeFollowNTicks( stuffyDelayLeft.getValue() );
  motor.stop_all();
}

void delayNTicks( int numTicks ) {
  long startTicks = encoderRPos;
  while ( encoderRPos - startTicks < numTicks ) {
    if ( debounceStuffyPin() ) {
      arduinoLeftStop();
    }
  }
}

void liftNTicks( int numTicks ) {

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
  delayNTicks(bridge2Back.getValue());
  motor.stop_all();
  RCServo0.write(0);
  RCServo0.write(0);
  delay(1000);

  // back up for a short distance to completely lay down the bridge
  motor.speed(motorLeft, -120);
  motor.speed(motorRight, -120);
  delayNTicks(backupdel2.getValue());

  motor.speed(motorLeft, 120);
  motor.speed(motorRight, 120);
  delayNTicks(bridge2For.getValue());

  motor.speed(motorLeft, 120 + turn2Offset.getValue());
  motor.speed(motorRight, 120 );
  delayNTicks(turn2del.getValue());
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
    LCD.setCursor(2, 0);
    LCD.print("detected high");
    delay(15);
    if (digitalRead(stuffyComPin) == HIGH ) {
      return true;
    }
  }

  return false;
}

bool debounceLowStuffyPin() {
  if ( digitalRead(stuffyComPin) == LOW ) {
    delay(15);
    if (digitalRead(stuffyComPin) == LOW ) {
      return false;
    }
  }

  return true;
}

void doYouEvenLiftBro( int numTicks ) {
  int speedVal = upSpeed.getValue();
  motor.speed(motorLift, -speedVal);
  long startTicks = encoderLiftPos;
  while ( encoderLiftPos - startTicks < numTicks ) {
    int startEncoderPos = encoderLiftPos;
    double currentTime = millis();
    while ( millis() - currentTime < 300 ) {
      LCD.clear(); LCD.home();
      LCD.print(encoderLiftPos - startTicks);
      if (encoderLiftPos - startTicks >= numTicks) {
        break;
      }
    }
    if ( startEncoderPos == encoderLiftPos) {
      speedVal += 20;
    }
  }
  motor.speed(motorLift, 0);
}

void doYouEvenLowerBro( int numTicks ) {
  int speedVal = downSpeed.getValue();
  long startTicks = encoderLiftPos;

  while ( encoderLiftPos - startTicks < numTicks ) {
    motor.speed(motorLift, speedVal);

    int startEncoderPos = encoderLiftPos;
    double currentTime = millis();

    while ( millis() - currentTime < 300 ) {
      LCD.clear(); LCD.home();
      LCD.print(encoderLiftPos - startTicks);
      if (encoderLiftPos - startTicks >= numTicks) {
        break;
      }
    }
    if ( startEncoderPos == encoderLiftPos) {
      speedVal += 20;
    }
  }
  motor.speed(motorLift, 0);
}

void suspensionBridge() {

  long startTicks = encoderRPos;
  while ( encoderRPos - startTicks < susdelay.getValue() ) {
    if (digitalRead(stuffyComPin) == HIGH) {
      arduinoStop();
    }
    edgePID.edgeFollow();
  }
}

void grandFinale() {
  while (true) {
    motor.speed(motorLeft, 120);
    motor.speed(motorRight, 120);
    if ( digitalRead(stuffyComPin) == HIGH ) {
      arduinoStop();
      doYouEvenLiftBro( lift2.getValue() );
      delayNTicks( dropdel2.getValue() );
      doYouEvenLowerBro( lower2.getValue() );
    }
    if ( pid.isEdge() ) {
      break;
    }
  }
}


