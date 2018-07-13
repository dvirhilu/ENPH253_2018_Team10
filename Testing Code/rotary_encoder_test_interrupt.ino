//#include <phys253.h>
//#include <LiquidCrystal.h>

/* read a rotary encoder with interrupts
   Encoder hooked up with common to GROUND,
   encoder0PinA to pin 2, encoder0PinB to pin 3 
   it doesn't matter which encoder pin you use for A or B

   uses Arduino pull-ups on A & B channel outputs
   turning on the pull-ups saves having to hook up resistors
   to the A & B channel outputs

*/

// References: 
// https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
// https://playground.arduino.cc/Main/RotaryEncoders

#define encoder0PinA  2
#define encoder0PinB  3
#define clicks_per_revolution  40

volatile int encoder0Pos = 0;
int current_time;
int start_time = 0;
int previous_pos;
float rpm;
int period = 500; // in millisecond

void doEncoder() {
  /* If pinA and pinB are both high or both low, it is spinning
     forward. If they're different, it's going backward.
  */
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    // clockwise
    encoder0Pos++;
  } else {
    // counterclockwise
    encoder0Pos--;
  }

  // uncomment the following two lines of code if you want to print the current position of the encoder
  Serial.println("Position: ");
  Serial.println(encoder0Pos, DEC);
}

void setup() {
  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA, HIGH);       // turn on pull-up resistor
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinB, HIGH);       // turn on pull-up resistor

  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoder, CHANGE);  // encoder pin on interrupt 0 - pin 2
  Serial.begin (9600);
  Serial.println("start");                // a personal quirk
  current_time = millis();
  previous_pos = encoder0Pos;
}

void loop() {
 
  // calculate rpm once every period
  if(millis() - current_time >= period) {
    rpm = 60.0 * ((encoder0Pos - previous_pos) / float(clicks_per_revolution))/(period / 1000.0);
    
    // uncomment the following two lines of code if you want to print the rpm
    //Serial.println("RPM: ");
    //Serial.println(rpm);
    
    current_time = millis();
    previous_pos = encoder0Pos;
  }
  
}


/*  to read the other two transitions - just use another attachInterrupt()
  in the setup and duplicate the doEncoder function into say,
  doEncoderA and doEncoderB.
  You also need to move the other encoder wire over to pin 3 (interrupt 1).
*/
