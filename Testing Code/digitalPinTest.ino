void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
pinMode(2, INPUT);
pinMode(3, INPUT);
pinMode(4, INPUT);
pinMode(5, INPUT);
pinMode(6, INPUT);
pinMode(7, INPUT);
pinMode(8, INPUT);
pinMode(9, INPUT);
pinMode(10, INPUT);
pinMode(11, INPUT);
pinMode(12, INPUT);
pinMode(13, INPUT);
}

void loop() {

  Serial.println( String(" pin 2: ") + digitalRead(2) + String( " pin 3: ") + digitalRead(3) + String(" pin 4: ") + digitalRead(4) + String(" pin 5: ") + digitalRead(5) + String(" pin 6: ") + digitalRead(6) + String(" pin 7: ") + digitalRead(7) + String(" pin 8: ") + digitalRead(8) + String(" pin 9: ") + digitalRead(9) +  String(" pin 10: ") + digitalRead(10) +  String(" pin 11: ") + digitalRead(11) + String(" pin 12: ") + digitalRead(12) + String(" pin 13: ") + digitalRead(13) ); 

}
