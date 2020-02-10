#include <SharpIR.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define potPin A0
#define trigPin 3
#define echoPin 2
#define IRPin A1
#define model 1080
#define STEPPER_STP_PIN 10
#define STEPPER_DIR_PIN 11
#define DCMOTOR_IN1 5
#define DCMOTOR_IN2 6

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// Create variable to store the distance:
int IRdistance_cm, setDist;
long duration, USdistance_cm;

/* IR Sensor Model :
  GP2Y0A02YK0F --> 20150
  *GP2Y0A21YK0F --> 1080
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

// DC MOTOR HELPER FUNCTIONS

// Create a new instance of the SharpIR class:
SharpIR IRsensor = SharpIR(IRPin, model);
Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver();

int val = 0;
float k = 0.1;

void setup() {
  // Begin serial communication at a baudrate of 9600:
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // STEPPER PINS
  pinMode(STEPPER_STP_PIN, OUTPUT);
  pinMode(STEPPER_DIR_PIN, OUTPUT);

  // SERVO PINS
  servo.begin();
  servo.setOscillatorFrequency(27000000);
  servo.setPWMFreq(SERVO_FREQ);

  // DC MOTOR PINS
  pinMode(DCMOTOR_IN1, OUTPUT);
  pinMode(DCMOTOR_IN2, OUTPUT);
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

//  if(USDiff <= 2 && IRDiff > 2) // if US Sensor is within 2cm and IR Sensor isn't -> Green LED
//  {
//    digitalWrite(greenLED,HIGH);
//    digitalWrite(redLED,LOW);
//  }
//  
//  if(IRDiff <= 2 && USDiff > 2) // if IR Sensor is within 2cm and US Sensor isn't -> Red LED
//  {
//    digitalWrite(greenLED,LOW);
//    digitalWrite(redLED,HIGH);
//  }
//  
//  if(USDiff <= 2 && IRDiff <= 2) // if they're both within 2cm -> both red and green LEDs
//  {
//    digitalWrite(greenLED,HIGH);
//    digitalWrite(redLED,HIGH);
//  }
//
//  if(USDiff > 2 && IRDiff > 2) // if they're both > 2cm -> both LEDs off
//  {
//    digitalWrite(greenLED,LOW);
//    digitalWrite(redLED,LOW);
//  }

  servo_goto(100);
//  analogWrite(DCMOTOR_IN1, LOW);
//  digitalWrite(DCMOTOR_IN2, LOW);
    
}

long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2;
}

void servo_goto(int deg)
{
  int pulselen = map(deg, 0, 180, SERVOMIN, SERVOMAX);
  uint8_t servonum = 15; // address on the board where the servo is wired
  servo.setPWM(servonum, 0, pulselen);
}
