#include <SharpIR.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define potPin A0
#define trigPin 2
#define echoPin 3
#define IRPin A1
#define model 1080
#define redLED 5
#define greenLED 6

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
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
//  pinMode(13,OUTPUT);
  servo.begin();
  servo.setOscillatorFrequency(27000000);
  servo.setPWMFreq(SERVO_FREQ);
 }

void loop() {
  // Get a distance measurement and store it as distance_cm:
  IRdistance_cm = IRsensor.distance();
  setDist = map(analogRead(potPin),0,1023,10,50);

  USdistance_cm = read_ultrasonic_sensor();
     
  // Print the measured distance to the serial monitor:
  
//  Serial.print("Set Distance: ");
  
//  Serial.println(" cm");
//  Serial.print("IR distance: ");
  Serial.print(IRdistance_cm);
  Serial.print(",");
//  Serial.println(" cm");
//  Serial.print("Ultrasonic distance: ");
  Serial.print(USdistance_cm);
  Serial.print(",");
//  Serial.println(" cm");
  Serial.println(setDist);
//  Serial.println();
//  Serial.println();
    
}

int read_ultrasonic_sensor() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  return microsecondsToCentimeters(duration);
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

// PID Control
int pos_last, pos_accumul;
int vel_last, vel_accumulate;

void limit(int *val, int minimum, int maximum) {
  if (*val > maximum) {
    *val = maximum;
  } else if (*val < minimum) {
    *val = minimum;
  }
}

int pid(int target, int current, int p, int i, int d, int i_clamp, int out_clamp, int deadzone, int punch, int *last, int *accumulate_error) {
  int error = target - current;

  // P stuff
  int p_term = p * error;

  // I stuff
  *accumulate_error += i * error;
  limit(accumulate_error, -i_clamp, i_clamp);
  int i_term = *accumulate_error * i;

  // D stuff
  int d_term  = (*last - current) * d;
  *last = current;

  // Punch stuff
  int punch_term = 0;
  if (error > deadzone) {
    punch_term = punch;
  } else if (error < deadzone) {
    punch_term = -punch;
  }

  int sum = p_term + i_term + d_term + punch_term;
  limit(&sum, -out_clamp, out_clamp);
  return sum;
}
