#include <phys253.h>
#include <LiquidCrystal.h>
// This is a test program for the two EN11 rotary encoders used for the robot of Team 10 
// See the link: http://www.hobbytronics.co.uk/arduino-tutorial6-rotary-encoder for details 

// Setting up the initial counter reading
int reading = 0;
// Pulses per revolution for EN11 
int lowest = -20;
int highest = 20;
//change of one pulse 
int changeamnt = 1;

// Timing for polling the encoder
unsigned long currentTime;
unsigned long lastTime;


// Pin Reference for left and right encoders
const int pinA = 0;
const int pinB = 1;

// Storing the readings
boolean encA;
boolean encB;
boolean lastL = false;

void setup() {
  // set the two digital pins as inputs 
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  // Set up the timing of the polling
  currentTime = millis();
  lastTime = currentTime; 
  // Start the serial monitor for debugging
  Serial.begin(9600);
} 


void loop()
{
  // Read elapsed time
  currentTime = millis();
  
  // 10 ms since last check of encoder = 100 Hz
  if(currentTime >= (lastTime + 10))
  {
    // read the two pins
    encA = digitalRead(pinA);
    encB = digitalRead(pinB);

    // Printing out the encoder values A and B
    Serial.println(encA);
    //Serial.print(",");
    //Serial.println(encB);
    
    
    // check if A has gone from high to low
    if ((!encA) && (lastA))
    {
      // check if B is high 
      if (encB)
      {
        // clockwise
        if (reading + changeamnt <= highest)
        {
          reading = reading + changeamnt; 
        }
      }
      else
      {
        // counterclockwise
        if (reading - changeamnt >= lowest)
        {
          reading = reading - changeamnt; 
        }
      }
      // Output reading for debugging
      //Serial.println(reading);
    }
    // store reading of A and millis for next loop
    lastA = encA;
    lastTime = currentTime;

  }

}
