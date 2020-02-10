#include <SharpIR.h>
// Define model and input pin:
#define potPin A0
#define trigPin 3
#define echoPin 2
#define IRPin A1
#define model 1080
#define redLED 5
#define greenLED 6
#define STEPPER_STP_PIN 10
#define STEPPER_DIR_PIN 11

// Create variable to store the distance:
int IRdistance_cm, setDist;
long duration, USdistance_cm;

/* IR Sensor Model :
  GP2Y0A02YK0F --> 20150
  GP2Y0A21YK0F --> 1080
  GP2Y0A710K0F --> 100500
  GP2YA41SK0F --> 430
*/

// STEPPER HELPER FUNCTIONS
void stepper_set_steps(int steps){
  if (steps < 0){
    digitalWrite(STEPPER_DIR_PIN, LOW);
    steps = steps * -1;
  }
  else{
    digitalWrite(STEPPER_DIR_PIN, HIGH);
  }

  for (int i = 0; i < steps; i++) {
    // These four lines result in 1 step:
    digitalWrite(STEPPER_STP_PIN, HIGH);
    delayMicroseconds(100);
    digitalWrite(STEPPER_STP_PIN, LOW);
    delayMicroseconds(100);
  }

}

// Create a new instance of the SharpIR class:
SharpIR IRsensor = SharpIR(IRPin, model);
int val = 0;
float k = 0.1;
void setup() {
  // Begin serial communication at a baudrate of 9600:
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  
  // STEPPER PINS
  pinMode(STEPPER_STP_PIN, OUTPUT);
  pinMode(STEPPER_DIR_PIN, OUTPUT);
}

void loop() {
  // Get a distance measurement and store it as distance_cm:
  IRdistance_cm = IRsensor.distance();
  setDist = map(analogRead(potPin),0,1023,10,50);

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  USdistance_cm = microsecondsToCentimeters(duration);
     
  // Print the measured distance to the serial monitor:
  Serial.print(IRdistance_cm);
  Serial.print(",");
  Serial.print(USdistance_cm);
  Serial.print(",");
  Serial.println(setDist);

  int USDiff = abs(setDist - USdistance_cm);
  int IRDiff = abs(setDist - IRdistance_cm);

  if(USDiff <= 2 && IRDiff > 2) // if US Sensor is within 2cm and IR Sensor isn't -> Green LED
  {
    digitalWrite(greenLED,HIGH);
    digitalWrite(redLED,LOW);
  }
  
  if(IRDiff <= 2 && USDiff > 2) // if IR Sensor is within 2cm and US Sensor isn't -> Red LED
  {
    digitalWrite(greenLED,LOW);
    digitalWrite(redLED,HIGH);
  }
  
  if(USDiff <= 2 && IRDiff <= 2) // if they're both within 2cm -> both red and green LEDs
  {
    digitalWrite(greenLED,HIGH);
    digitalWrite(redLED,HIGH);
  }

  if(USDiff > 2 && IRDiff > 2) // if they're both > 2cm -> both LEDs off
  {
    digitalWrite(greenLED,LOW);
    digitalWrite(redLED,LOW);
  }

  stepper_set_steps(100);
    
}

long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2;
}
