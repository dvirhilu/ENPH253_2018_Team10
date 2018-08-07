#include <FrequencyDetection.h>

int tenkHzPin = 3;
int onekHzPin = 2;
double tenReading = 0;
double oneReading = 0;
FrequencyDetection freq = FrequencyDetection(tenkHzPin, onekHzPin, 6);

void setup() {  // put your setup code here, to run once:
  Serial.begin(9600);
  freq.set1kHzThresh(250);
  freq.set10kHzThresh(65);
}

void loop() {
  // put your main code here, to run repeatedly
  Serial.print(freq.is1kHz());
  Serial.print(" ");
  Serial.println(freq.is10kHz());
  delay(50);
}
