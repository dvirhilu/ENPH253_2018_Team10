#include <phys253.h>
#include <LiquidCrystal.h>
#include <BinLift.h>
#include <avr/interrupt.h>  

int lift_motor_pin = 1;
int lift_speed = -200;
int lower_speed = 75;
int currentPos = 0;
int bin1HeightClicks = 105;
int bin2HeightClicks = 80;

BinLift binlift(lift_motor_pin);

enum PinAssignments {
  encoderPinA = 2,   // right
};

volatile unsigned int encoderPos = 0;  // a counter for the dial
boolean rotating = false;    // debounce management
boolean set = false;
ISR(INT2_vect) {encode();}

/*  Enables an external interrupt pin
INTX: Which interrupt should be configured?
    INT0    - will trigger ISR(INT0_vect)
    INT1    - will trigger ISR(INT1_vect)
    INT2    - will trigger ISR(INT2_vect)
    INT3    - will trigger ISR(INT3_vect)
mode: Which pin state should trigger the interrupt?
    LOW     - trigger whenever pin state is LOW
    FALLING - trigger when pin state changes from HIGH to LOW
    RISING  - trigger when pin state changes from LOW  to HIGH 
*/
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

void encode() {
  // debouncing 
  delayMicroseconds(30);
  if ( digitalRead(encoderPinA) != set ) {
    encoderPos++;
    Serial.println(encoderPos);
  }
}


void setup() {
  #include <phys253setup.txt>
  enableExternalInterrupt(INT2, RISING);
  pinMode(encoderPinA, INPUT);
  digitalWrite(encoderPinA, HIGH);
  binlift.setDefaultSpeed(lift_speed, lower_speed);
  
  Serial.begin(9600);
}

void loop() {
  Serial.print("Index:");
  Serial.println(encoderPos, DEC);
  currentPos = encoderPos;
  while(encoderPos - currentPos < bin1HeightClicks) {
    Serial.print("Raise:");
    Serial.println(encoderPos, DEC);
    binlift.binLift();
  }
  
  motor.speed(lift_motor_pin, 0);

  // continue driving forward for a while till the zipline has passed 
  start_time = millis();
  current_time = millis() - start_time;
  while(current_time < 4000){
    pid.tapeFollow();
    current_time = millis() - start_time;
  }

  currentPos = encoderPos;

  while(encoderPos - currentPos < bin1HeightClicks) {
    binlift.binLower();
  }
  
  motor.speed(lift_motor_pin, 0);
  encoderPos = 0;
  currentPos = encoderPos;
 
}

