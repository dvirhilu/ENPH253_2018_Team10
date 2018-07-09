#include "Arduino.h"
#include "StuffyRescue.h"
#include "Servo.h"

StuffyRescue::StuffyRescue(int limitSwitch, int arm, int claw, int sensor, int LED){
	switchPin = limitSwitch;
	armServoPin = arm;
	clawServoPin = claw;
	sensorPin = sensor;
	LEDPin = LED;
	pinMode(switchPin, INPUT);
	pinMode(LEDPin, OUTPUT);
	armServo.attach(armServoPin);
	clawServo.attach(clawServoPin);
}

 void StuffyRescue::setup(int armPos, int clawPos) {
	armServo.write(armPos);
	while(armServo.read() > armPos){
		
	}
	clawServo.write(clawPos);
	state = StuffyState::search;
  return;
}

void StuffyRescue::poll(int threshold) {
	// continuously reads IR sensor until reading is above the threshold
	if(state == StuffyState::search){
		if(analogRead(sensorPin) > threshold){
		state = StuffyState::detected;
		}
	}
return;
}

void StuffyRescue::pickup(int closedPos, int loweredPos, int armInitialPos, int clawInitialPos) {
	if(state == StuffyState::detected){
		armServo.write(100); // bring servo arm down
		while(armServo.read() > loweredPos){
		}
		clawServo.write(100); // close claw servo
		while (clawServo.read() > closedPos){
		}
		if (digitalRead(switchPin)==LOW){
		state = StuffyState::captured;
		setup(armInitialPos, clawInitialPos);
		}
		else{
		state = StuffyState::fuckedUp;
		}
	}
return;
}

