#include <SharpIR.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>

#define potPin A0
#define trigPin 5
#define echoPin 4
#define IRPin A1
#define model 1080
#define STEPPER_STP_PIN 12
#define STEPPER_DIR_PIN 13
#define DCMOTOR_IN1 5
#define DCMOTOR_IN2 6
#define SERVO_PIN 10

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
Servo servo;

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
  servo.attach(SERVO_PIN);

  // DC MOTOR PINS
  pinMode(DCMOTOR_IN1, OUTPUT);
  pinMode(DCMOTOR_IN2, OUTPUT);
 }

void loop() {
  // Get a distance measurement and store it as distance_cm:
  IRdistance_cm = IRsensor.distance();
  setDist = map(analogRead(potPin),0,1023,10,50);

  USdistance_cm = read_ultrasonic_sensor();
     
  // Print the measured distance to the serial monitor:
  Serial.print(IRdistance_cm);
  Serial.print(",");
  Serial.print(USdistance_cm);
  Serial.print(",");
  Serial.println(setDist);
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