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
  encoderPinB = 3,   // left
  clearButton = 8    // another two pins
};

volatile unsigned int encoderPos = 0;  // a counter for the dial
unsigned int lastReportedPos = 1;   // change management
static boolean rotating = false;    // debounce management

// interrupt service routine vars
boolean A_set = false;
boolean B_set = false;

// Interrupt on A changing state
void doEncoderA() {
  // debounce
  if ( rotating ) delay (1);  // wait a little until the bouncing is done

  // Test transition, did things really change?
  if ( digitalRead(encoderPinA) != A_set ) { // debounce once more
    A_set = !A_set;

    // adjust counter + if A leads B
    if ( A_set && !B_set )
      encoderPos += 1;

    rotating = false;  // no more debouncing until loop() hits again
  }
}

// Interrupt on B changing state, same as A above
void doEncoderB() {
  if ( rotating ) delay (1);
  if ( digitalRead(encoderPinB) != B_set ) {
    B_set = !B_set;
    //  adjust counter - 1 if B leads A
    if ( B_set && !A_set )
      encoderPos -= 1;

    rotating = false;
  }
}

ISR(INT2_vect) {doEncoderA(); doEncoderB();}
ISR(INT3_vect) {doEncoderA(); doEncoderB();}


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

 

void setup() {
  #include <phys253setup.txt>
  enableExternalInterrupt(INT2, RISING);
  enableExternalInterrupt(INT3, RISING);
  enableExternalInterrupt(INT2, FALLING);
  enableExternalInterrupt(INT3, FALLING);

  //portMode(0, INPUT) ;      //   ***** from 253 template file
  //portMode(1, INPUT) ;      //   ***** from 253 template file
  
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(clearButton, INPUT);
  // turn on pullup resistors
  digitalWrite(encoderPinA, HIGH);
  digitalWrite(encoderPinB, HIGH);
  digitalWrite(clearButton, HIGH);

  

  // encoder pin on interrupt 0 (pin 2)
  //attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderA, CHANGE);
  // encoder pin on interrupt 1 (pin 3)
  //attachInterrupt(digitalPinToInterrupt(encoderPinB), doEncoderB, CHANGE);

  Serial.begin(9600);  // output
}

// main loop, work is done by interrupt service routines, this one only prints stuff
void loop() {
  rotating = true;  // reset the debouncer

  if (lastReportedPos != encoderPos) {
    Serial.print("Index:");
    Serial.println(encoderPos, DEC);
    LCD.clear(); LCD.home();
    LCD.print("Index: ");
    LCD.print(encoderPos);
    lastReportedPos = encoderPos;
  }
  if (digitalRead(clearButton) == LOW )  {
    encoderPos = 0;
  }
}

