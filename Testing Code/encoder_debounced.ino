#include <phys253.h>
#include <LiquidCrystal.h>

#include <avr/interrupt.h>  

/* interrupt routine for Rotary Encoders
   tested with Noble RE0124PVB 17.7FINB-24 http://www.nobleusa.com/pdf/xre.pdf - available at pollin.de
   and a few others, seems pretty universal
   The average rotary encoder has three pins, seen from front: A C B
   Clockwise rotation A(on)->B(on)->A(off)->B(off)
   CounterCW rotation B(on)->A(on)->B(off)->A(off)
   and may be a push switch with another two pins, pulled low at pin 8 in this case
   raf@synapps.de 20120107
*/

// usually the rotary encoders three pins have the ground pin in the middle
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
  }
}

 

void setup() {
  #include <phys253setup.txt>
  enableExternalInterrupt(INT2, RISING);

  pinMode(encoderPinA, INPUT);
  // turn on pullup resistors
  digitalWrite(encoderPinA, HIGH);
  Serial.begin(9600);  // output
}

// main loop, work is done by interrupt service routines, this one only prints stuff
void loop() {
  Serial.print("Index:");
  Serial.println(encoderPos, DEC);
}

