/*
    Ultrasonic sensor Pins:
        VCC: +5VDC
        Trig : Trigger (INPUT) - Pin11
        Echo: Echo (OUTPUT) - Pin 12
        GND: GND
 */
 
int trigPin = 2;    //Trig - green Jumper
int echoPin = 3;    //Echo - yellow Jumperhttps://www.arduino.cc/en/uploads/Guide/Linux_Install_2.jpg
long duration, cm, inches;

// kalman variables
float varVolt = 1.12184278324081E-05;  // variance determined using excel and reading samples of raw sensor data
float varProcess = 1e-8;
float Pc = 0.0;
float G = 0.0;
float P = 1.0;
float Xp = 0.0;
float Zp = 0.0;
float Xe = 0.0;

 
void setup() {
  //Serial Port begin
  Serial.begin (9600);
  //Define inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}
 


void loop()
{
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  duration = pulseIn(echoPin, HIGH);
 
  // convert the time into a distance
  cm = (duration/2) / 29.1;
  inches = (duration/2) / 74; 

  // kalman process
  Pc = P + varProcess;
  G = Pc/(Pc + varVolt);    // kalman gain
  P = (1-G)*Pc;
  Xp = Xe;
  Zp = Xp;
  Xe = G*(cm-Zp)+Xp;   // the kalman estimate of the sensor voltage

//  if (Xe < 20){ // prevent overflowing
    Serial.println(Xe);
//  }
  delay(20);

/*
  Serial.println(cm);
  Serial.print(" ");
  8*/
  
  delay(20);
}


//Moving average filter
//
//
//
//
///*
//    Ultrasonic sensor Pins:
//        VCC: +5VDC
//        Trig : Trigger (INPUT) - Pin11
//        Echo: Echo (OUTPUT) - Pin 12
//        GND: GND
// */
// 
//int trigPin = 2;    //Trig - green Jumper
//int echoPin = 3;    //Echo - yellow Jumperhttps://www.arduino.cc/en/uploads/Guide/Linux_Install_2.jpg
//long duration, cm, inches;
//
//const int numReadings = 10;
//
//float readings[numReadings];      // the readings from the analog input
//int readIndex = 0;              // the index of the current reading
//float total = 0;                  // the running total
//float average = 0;                // the average
//
// 
//void setup() {
//  //Serial Port begin
//  Serial.begin (9600);
//  //Define inputs and outputs
//  pinMode(trigPin, OUTPUT);
//  pinMode(echoPin, INPUT);
//}
// 
//
//
//void loop()
//{
//  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
//  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
//  digitalWrite(trigPin, LOW);
//  delayMicroseconds(5);
//  digitalWrite(trigPin, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(trigPin, LOW);
// 
//  // Read the signal from the sensor: a HIGH pulse whose
//  // duration is the time (in microseconds) from the sending
//  // of the ping to the reception of its echo off of an object.
//  pinMode(echoPin, INPUT);
//  duration = pulseIn(echoPin, HIGH);
// 
//  // convert the time into a distance
//  cm = (duration/2) / 29.1;
//  inches = (duration/2) / 74; 
//  /*
//  Serial.print(inches);
//  Serial.print("in, ");
//  Serial.print(cm);
//  Serial.print("cm");
//  Serial.println();
//  */
//
//  // subtract the last reading:
//  total = total - readings[readIndex];
//  // read from the sensor:
//  readings[readIndex] = cm;
//  // add the reading to the total:
//  total = total + readings[readIndex];
//  // advance to the next position in the array:
//  readIndex = readIndex + 1;
//
//  // if we're at the end of the array...
//  if (readIndex >= numReadings) {
//    // ...wrap around to the beginning:
//    readIndex = 0;
//  }
//
//  // calculate the average:
//  average = total / numReadings;
//  // send it to the computer as ASCII digits
//  Serial.println(average);
//
///*
//  Serial.println(cm);
//  Serial.print(" ");
//  8*/
//  delay(20);
//}
//
//
