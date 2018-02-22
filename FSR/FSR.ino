int FSRPIN = A2;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // print the sensor values:
  Serial.print(analogRead(FSRPIN));
  Serial.println();
  // delay before next reading:
  delay(20);

}
