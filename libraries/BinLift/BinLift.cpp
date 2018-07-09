#include "Arduino.h"
#include "BinLift.h"
#include "NewPing.h"

BinLift::BinLift(int motor,int sonar, int thresh, int encoder){
	motorPin = motor;
	sonarPin = sonar;
	sonarThresh = thresh;
	endcoderPin = encoder;
	pinMode(encoderPin, INPUT);
	pinMode(QRDPin, INPUT);
}

 void BinLift::raise() {
	
return;
}

void BinLift::lower() {
	
return;
}

void BinLift::poll(){
	if(digitalRead(sonarPin) > sonarThresh){
		state = BinState::detected;
	}
return;
}

