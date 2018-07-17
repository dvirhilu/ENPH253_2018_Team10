#include <phys253.h>
#include <LiquidCrystal.h>

int overheadCount = 0;
long duration; 
int distance;
double pot_reading; // reading from the potentiometer
int angle; // angle position of the potentiometer
double distance_sonar_arch = 35; // in cm

volatile unsigned int trigPin = 8; // digital pin #5 on TINAH for Sonar Trig Output Pin
volatile unsigned int echoPin = 0; // digital pin #4 on TINAH for Sonar Echo Input Pin
volatile unsigned int PotPin = 0;
volatile unsigned int initialAngle = 90;
volatile unsigned int maxAngle = 270;
volatile unsigned int bin1Angle = initialAngle + 45;
volatile unsigned int bin2Angle = initialAngle + 90;
int start_time; 
int current_time;

double sonarThreshold = (distance_sonar_arch * 2 / 343) * 1000000; 
int liftingSpeed = 100; // the speed to lift the bins


void setup() {
  #include <phys253setup.txt>
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

}

void loop() {
  LCD.clear(); LCD.home();
  LCD.print("Bin Lifting Test");
  LCD.setCursor(0, 1);
  LCD.print("press <start>");
  while(!(startbutton())){};
  
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
      
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");
  } 

  overheadCount = 0;
  while (!(stopbutton()) && !(startbutton())) {

    
    while (distance >= distance_sonar_arch && overheadCount == 0) {
      PID();

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
      distance= duration*0.0343/2;
      
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");
      
      // passed the archway 
      if(distance <= distance_sonar_arch) {
        overheadCount = 1;
        delay(300); // let the whole robot pass the arch
        //motor.stop_all();
        binLift1();
        
        start_time = millis();
        current_time = millis() - start_time;
        // drive forward for 4000 milliseconds before lowering the bin
        while(current_time < 4000){
          PID();
          current_time = millis() - start_time;
        }
        
        binLower();
        break;
      }
      
    }
    PID();
   
  }

}

void PID() {
  LCD.clear(); LCD.home();
  LCD.print("Tape Following");
  LCD.setCursor(0, 1);
  LCD.print("PID() Mode");
  delay(1000);
}

void binLift1() {
  pot_reading = analogRead(PotPin);
  angle = (pot_reading / 1000) * maxAngle;
  
  while(angle < bin1Angle ) {
    pot_reading = analogRead(PotPin);
    angle = (pot_reading / 1000) * maxAngle;
    LCD.clear(); LCD.home();
    LCD.print("Bin Lifting");
    LCD.setCursor(0, 1);
    LCD.print(angle);
    LCD.print(" --> ");
    LCD.print(bin1Angle);
  }

}


void binLift2() {
  pot_reading = analogRead(PotPin);
  angle = (pot_reading / 1000) * maxAngle;
  
  while(angle < bin2Angle ) {
    pot_reading = analogRead(PotPin);
    angle = (pot_reading / 1000) * maxAngle;
    LCD.clear(); LCD.home();
    LCD.print("Bin Lifting");
    LCD.setCursor(0, 1);
    LCD.print(angle);
    LCD.print(" --> ");
    LCD.print(bin2Angle);
  }
}

void binLower() {
  pot_reading = analogRead(PotPin);
  angle = (pot_reading / 1000) * maxAngle;
  
  while(angle > initialAngle ) {
    pot_reading = analogRead(PotPin);
    angle = (pot_reading / 1000) * maxAngle;
    
    LCD.clear(); LCD.home();
    LCD.print("Bin Lowering");
    LCD.setCursor(0, 1);
    LCD.print(angle);
    LCD.print(" --> ");
    LCD.print(initialAngle);
  }
}



