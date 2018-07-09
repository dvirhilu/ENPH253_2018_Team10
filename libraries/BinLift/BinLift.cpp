#include "Arduino.h"
#include "BinLift.h"

BinLift::BinLift(int motor,int QRD, int thresh, int encoder){
	motorPin = motor;
	QRDPin = QRD;
	QRDthresh = thresh;
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
	if(analogRead(QRDPin) > QRDthresh){
		state = BinState::detected;
	}
return;
}

