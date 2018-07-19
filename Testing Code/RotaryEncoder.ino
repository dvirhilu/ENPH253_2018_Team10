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
#define clicks_per_revolution  200

volatile int encoder0Pos = 0;
double previous_time;
double current_time;
double start_time = 0;
int previous_pos;
float freqHz;
int period = 1000; // in microseconds
int rotations = 0;
int count = 0;

/*
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
  //Serial.println("Position: ");
  //Serial.println(encoder0Pos, DEC);
}*/


// updates the number of clicks each time the interrupt pin signals HIGH
// By diregarding the information about the direction of rotation we can use only one digital pin for the encoder
void encode() {
  encoder0Pos++;
}

void setup() {
  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA, HIGH);       // turn on pull-up resistor
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinB, HIGH);       // turn on pull-up resistor

  attachInterrupt(digitalPinToInterrupt(encoder0PinA), encode, RISING);  // encoder pin on interrupt 0 - pin 2
  Serial.begin (9600);
  Serial.println("start");                // a personal quirk
  previous_time = micros()/1000000.0;
  previous_pos = encoder0Pos;
}

void loop() {

  current_time = micros()/1000000.0;
  // calculate freqHz once every period
  if (current_time - previous_time >= period/1000000.0) {
    freqHz += (encoder0Pos - previous_pos) / ((double)clicks_per_revolution * (current_time - previous_time));
    rotations = encoder0Pos / clicks_per_revolution;
    count++;
    
    previous_time = current_time;
    previous_pos = encoder0Pos;
  }

  if (count == 50) {
    freqHz = freqHz / count;

     Serial.print(previous_time);
    Serial.print("\t");
    Serial.print(current_time);
    Serial.print("\t");
    Serial.print("freqHz: ");
    Serial.print("\t");
    Serial.print(freqHz);
    Serial.print("\t");
    Serial.print(rotations);
    Serial.print("\t");
    Serial.println(encoder0Pos);

    count = 0;
  }

}


/*  to read the other two transitions - just use another attachInterrupt()
  in the setup and duplicate the doEncoder function into say,
  doEncoderA and doEncoderB.
  You also need to move the other encoder wire over to pin 3 (interrupt 1).
*/
