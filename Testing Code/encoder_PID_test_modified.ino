
#include <phys253.h>
#include <LiquidCrystal.h>
#include<MenuItem.h>
#include<avr/EEPROM.h>
#include <encoderPID.h>
#include <avr/interrupt.h>  

//PID values
constexpr int motorLeft = 2;
constexpr int motorRight = 0;

constexpr int encoderLeftPin = 0;
constexpr int encoderRightPin = 1;
constexpr int a_knob_thresh = 100;
constexpr int s_knob_thresh = 270;
constexpr int p_knob_thresh = 200;
constexpr int stuffyComPin = 12;

double current_time;
double prev_time;
double prevTime;
int k_p;
int k_d;
int derivative;
int gain;
int l_rpm;
int r_rpm;                  
int left_default_speed = 20;
int right_default_speed = 20;

double prev_error; 
double error;
int pid;
int cpr = 25;

int lrpmThreshold = 100;
int rrpmThreshold = 100;

enum PinAssignments {
  encoderPinR = 0,   // right
  encoderPinL = 1,   // left
};

volatile unsigned int encoderLPos = 0;  // a counter for the dial
volatile unsigned int encoderRPos = 0;  // a counter for the dial
volatile unsigned int prevEncoderLPos = 0;  
volatile unsigned int prevEncoderRPos = 0; 
volatile unsigned int currentLPos = 0;  
volatile unsigned int currentRPos = 0; 


ISR(INT0_vect) {encodeR();}
ISR(INT1_vect) {encodeL();}

encoderPID enpid( encoderLeftPin, encoderRightPin, motorLeft, motorRight );

void enableExternalInterrupt(unsigned int INTX, unsigned int mode) {
  if (INTX > 3 || mode > 3 || mode == 1) return;
  cli();
  /* Allow pin to trigger interrupts        */
  EIMSK |= (1 << INTX);
  /* Clear the interrupt configuration bits */
  EICRA &= ~(1 << (INTX*2+0));
  EICRA &= ~(1 << (INTX*2+1));
  /* Set new interrupt configuration bits   */
  EICRA |= mode << (INTX*2);
  sei();
}

void encodeL() {
  // debouncing 
  delayMicroseconds(30);
  if ( digitalRead(encoderPinL) == true ) {
    encoderLPos++;
  }
}

void encodeR() {
  // debouncing 
  delayMicroseconds(30);
  if ( digitalRead(encoderPinR) == true ) {
    encoderRPos++;
  }
}



MenuItem kp = MenuItem("p_k_p", (unsigned int*)53);
MenuItem kd = MenuItem("p_k_d", (unsigned int*)57);
MenuItem gainz = MenuItem("p_gainzz", (unsigned int*)61);
MenuItem lrpm = MenuItem("a_leftrpm", (unsigned int*)65);
MenuItem rrpm = MenuItem("a_rightrpm", (unsigned int*)69);
//MenuItem motor_speed = MenuItem("m_speeeeed", (unsigned int*)73);
MenuItem menu[] = {kp, kd, gainz, lrpm, rrpm};

char analog_sensors = 'a';
char servos = 's';
char PID_constants = 'p';

void setup() {
  #include <phys253setup.txt>
  enableExternalInterrupt(INT0, RISING);
  enableExternalInterrupt(INT1, RISING);

  pinMode(encoderPinL, INPUT);
  pinMode(encoderPinR, INPUT);
  // turn on pullup resistors
  digitalWrite(encoderPinL, HIGH);
  digitalWrite(encoderPinR, HIGH);
  
  Serial.begin(9600);
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

  k_p = kp.getValue();
  k_d = kd.getValue();
  gain = gainz.getValue();
  l_rpm = lrpm.getValue();
  r_rpm = rrpm.getValue();
  //enpid.setDefaultSpeed( motor_speed.getValue() );

  LCD.clear(); LCD.home();
  LCD.print("REEEEEEEEEE");
  delay(1000);


  while (!(stopbutton()) && !(startbutton())) {

    current_time = micros();
    
    currentLPos = encoderLPos;
    currentRPos = encoderRPos;
    
    int rpmR = (currentRPos - prevEncoderRPos) / (25 * (current_time - prev_time)/ 1000000) * 60.0;
    int rpmL = (currentLPos - prevEncoderLPos) / (25 * (current_time - prev_time)/ 1000000) * 60.0;
    
    goStraight();
    Serial.print("lrpm = ");
    Serial.print(l_rpm);
    Serial.print("  rrpm = ");
    Serial.println(r_rpm);

    prev_time = current_time;
    prevEncoderLPos = currentLPos;
    prevEncoderRPos = currentRPos;
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
double getLeftPID() {

    l_rpm += ((currentLPos - prevEncoderLPos) / (cpr * ((current_time - prev_time)/1000000.0))) * 60.0 ;
    countLeft++;

    double error = 0;
    double derivative = 0;
    double pid;
    
    if( countLeft > 10){
        l_rpm = l_rpm/countLeft;
        error = lrpmThreshold - l_rpm;
        countLeft = 0;
        derivative = (error - prev_error) / (current_time - prevTime);
        LCD.clear();LCD.home();
        LCD.print((int)l_rpm + String(" ") + error + String( " "));
        prev_error = error;
        prevTime = prev_time;
    }
    LCD.setCursor(0, 1);
    pid = gain * (k_p / 10.0 * error + k_d / 10.0 * derivative);
    return pid;
}
*/
double getLeftPID() {
  l_rpm = 0;
  for (int countLeft = 0; countLeft++; countLeft < 10) {
    current_time = micros();
    
    // update the current positions of the encoders
    currentLPos = encoderLPos;
    prevEncoderLPos = currentLPos;
    
    // calculate RPM
    l_rpm += ((currentLPos - prevEncoderLPos) / (cpr * ((current_time - prev_time)/1000000.0))) * 60.0 ;
    prev_time = current_time;
    
    delay(10);
  }

  l_rpm /= 10;
  error = lrpmThreshold - l_rpm;
  derivative = (error - prev_error) / (current_time - prevTime);
  LCD.setCursor(0, 0);
  LCD.print(String("LRPM=") + (int)l_rpm + String(" Err=") + error);
  prev_error = error;
  prevTime = current_time;
  pid = gain * (k_p / 10.0 * error + k_d / 10.0 * derivative);
  return pid;
}



double getRightPID() {
  r_rpm = 0;
  for (int countRight = 0; countRight++; countRight < 10) {
    current_time = micros();
    
    // update the current positions of the encoders
    currentRPos = encoderRPos;
    prevEncoderLPos = currentRPos;
    
    // calculate RPM
    r_rpm += ((currentRPos - prevEncoderRPos) / (cpr * ((current_time - prev_time)/1000000.0))) * 60.0 ;
    prev_time = current_time;
    
    delay(10);
  }

  r_rpm /= 10;
  error = rrpmThreshold - r_rpm;
  derivative = (error - prev_error) / (current_time - prevTime);
  LCD.setCursor(0, 1);
  LCD.print(String("RRPM=") + (int)r_rpm + String(" Err=") + error);
  prev_error = error;
  prevTime = current_time;
  pid = gain * (k_p / 10.0 * error + k_d / 10.0 * derivative);
  return pid;
}


/*
double getRightPID() {

    r_rpm += ((currentRPos - prevEncoderRPos) / (cpr * ((current_time - prev_time)/1000000.0))) * 60.0 ;
    countRight++;

    double error = 0;
    double derivative = 0;
    double pid;
    
    if( countRight > 10){
        r_rpm = r_rpm/countRight;
        error = rrpmThreshold - r_rpm;
        countRight = 0;
        derivative = (error - prev_error) / (current_time - prevTime);
        LCD.clear();LCD.home();
        LCD.print((int)r_rpm + String(" ") + error + String( " "));
        prev_error = error;
        prevTime = prev_time;
    }
    LCD.setCursor(0, 1);
    pid = gain * (k_p / 10.0 * error + k_d / 10.0 * derivative);
    return pid;
}
*/

void goStraight() {
  double leftPID;
  double rightPID;
      
  rightPID = getRightPID();
  leftPID = getLeftPID();
      
  left_default_speed+= leftPID;
  right_default_speed+= rightPID;
     
  motor.speed(motorLeft, left_default_speed);
  motor.speed(motorRight, right_default_speed);
}

