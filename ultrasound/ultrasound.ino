/*
    Ultrasonic sensor Pins:
        VCC: +5VDC
        Trig : Trigger (INPUT) - Pin11
        Echo: Echo (OUTPUT) - Pin 12
        GND: GND
 */
 
const int TRIGPIN1 = 2;
const int ECHOPIN1 = 3;
long duration, cm, inches;

//// kalman variables
//float varVolt = 1.12184278324081E-05;  // variance determined using excel and reading samples of raw sensor data
//float varProcess = 1e-8;
//float Pc = 0.0;
//float G = 0.0;
//float P = 1.0;
//float Xp = 0.0;
//float Zp = 0.0;
//float Xe = 0.0;

 
void setup() {
  //Serial Port begin
  Serial.begin (9600);
  //Define inputs and outputs
  pinMode(TRIGPIN1, OUTPUT);
  pinMode(ECHOPIN1, INPUT);
}
 


void loop()
{
  duration = readUS1();
 
  // convert the time into a distance
  cm = (duration/2) / 29.1;
  inches = (duration/2) / 74; 

//  // kalman process
//  Pc = P + varProcess;
//  G = Pc/(Pc + varVolt);    // kalman gain
//  P = (1-G)*Pc;
//  Xp = Xe;
//  Zp = Xp;
//  Xe = G*(cm-Zp)+Xp;   // the kalman estimate of the sensor voltage

  Serial.print(duration);
  Serial.print('\t');
  Serial.print(cm);
  Serial.println();
  delay(20);
}

// read ultrasound duration
long readUS1() {
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(TRIGPIN1, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGPIN1, HIGH);
  delayMicroseconds(5);
  digitalWrite(TRIGPIN1, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(ECHOPIN1, INPUT);
  return pulseIn(ECHOPIN1, HIGH, 20*1000);
}
