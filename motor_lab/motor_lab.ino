#include <SharpIR.h>
#include <Wire.h>
//#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>

#define potPin A0
#define trigPin 8
#define echoPin 7
#define IRPin A1
#define model 1080
#define STEPPER_STP_PIN 12
#define STEPPER_DIR_PIN 13
#define STEPPER_STEPS_PER_REV 400
#define SERVO_PIN 10

#define PWM_MAX 255

// Motor encoder pins
#define PIN_MOTOR_ENCODER_A 2
#define PIN_MOTOR_ENCODER_B 3

#define PIN_MOTOR_IN_1 5
#define PIN_MOTOR_IN_2 6

#define DCM_CONTROL_POSITION 1
#define DCM_CONTROL_VELOCITY 0 

// Motor encoder direction.
// Used for skip states by the encoder
// Thses values must be one or -1 for the code to function correctly
#define DCM_CC -1
#define DCM_CCW 1

// Ratio between encoder ticks and degrees
#define DCM_TICKS_TO_DEG_NUMERATOR 360.0
#define DCM_TICKS_TO_DEG_DENOMINATOR 3400.0

// Gains for position control for dc motor
#define POS_P 10
#define POS_I 0 
#define POS_D 0 
#define POS_I_CLAMP 65536
#define POS_OUT_CLAMP PWM_MAX
#define POS_PUNCH 0 
#define POS_DEADZONE 0 

// Gains for velocity control for dc motor
#define VEL_P 1
#define VEL_I 0 
#define VEL_D 0 
#define VEL_I_CLAMP 65536
#define VEL_OUT_CLAMP PWM_MAX
#define VEL_PUNCH 0 
#define VEL_DEADZONE 0 

// Returns the sign of a number
#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

// Create variable to store the distance:
int IRdistance_cm, setDist;
long USdistance_cm;

// Previous encoder state;
int dcm_fb_a;
int dcm_fb_b;
int dcm_fb_direction; 
int dcm_fb_position_ticks;

// DC Motor feedback
int dcm_fb_position, dcm_target_position;
int dcm_fb_velocity, dcm_target_velocity;
int dcm_velocity_power;
int dcm_control_scheme;

// PID Control history values
int pos_last, pos_accumulate;
int vel_last, vel_accumulate;

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
    delayMicroseconds(1000);
    digitalWrite(STEPPER_STP_PIN, LOW);
    delayMicroseconds(1000);
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
  pinMode(PIN_MOTOR_IN_1, OUTPUT);
  pinMode(PIN_MOTOR_IN_2, OUTPUT);

  pinMode(PIN_MOTOR_ENCODER_A, INPUT);
  pinMode(PIN_MOTOR_ENCODER_B, INPUT);

  // Attach interrupts for motor encoders
//  attachInterrupt(digitalPinToInterrupt(PIN_MOTOR_ENCODER_A), update_dcm_pos_feedback, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(PIN_MOTOR_ENCODER_B), update_dcm_pos_feedback, CHANGE);

  // Initialize encoder state
  dcm_fb_a = digitalRead(PIN_MOTOR_ENCODER_A);
  dcm_fb_b = digitalRead(PIN_MOTOR_ENCODER_B);
  
  // Zero out globals
  dcm_fb_position = 0;
  dcm_fb_position_ticks = 0;
  dcm_fb_velocity = 0;
  dcm_target_position = 180;
  dcm_target_velocity = 0;
  dcm_velocity_power = 0;
  dcm_control_scheme = DCM_CONTROL_POSITION;
 }

void loop() {
  // Update sensors
  IRdistance_cm = IRsensor.distance();
//  setDist = map(analogRead(potPin),0,1023,0,50);
  setDist = analogRead(potPin);
  USdistance_cm = read_ultrasonic_sensor();
  print_sensors();
//  dcm_step();
}

int read_ultrasonic_sensor() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  int duration = pulseIn(echoPin, HIGH);
  return microsecondsToCentimeters(duration);
}

void print_sensors() {
  // Print the measured distance to the serial monitor:
  Serial.print(IRdistance_cm);
  Serial.print(",");
  Serial.print(USdistance_cm);
  Serial.print(",");
  Serial.println(setDist);
  
//  stepper_set_steps(100);
}

void servo_test(){
  for (int pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (int pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    servo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2;
}

// Limits the value of *val to be minimum <= *val <= maximum
void limit(int *val, int minimum, int maximum) {
  if (*val > maximum) {
    *val = maximum;
  } else if (*val < minimum) {
    *val = minimum;
  }
}

// Operates dc motor
void dcm_step() {
  if (dcm_control_scheme == DCM_CONTROL_POSITION) {
    int pos_pid =  pid(dcm_target_position, dcm_fb_position, 
                    POS_P, POS_I, POS_D, 
                    POS_I_CLAMP, POS_OUT_CLAMP, POS_PUNCH, POS_DEADZONE, 
                    &pos_last, &pos_accumulate);

    if (pos_pid > 0) {
      analogWrite(PIN_MOTOR_IN_1, pos_pid);
      digitalWrite(PIN_MOTOR_IN_2, LOW);
    } else if (pos_pid < 0) {
      digitalWrite(PIN_MOTOR_IN_1, LOW);
      analogWrite(PIN_MOTOR_IN_2, abs(pos_pid));
    }
                    
  } else if (dcm_control_scheme == DCM_CONTROL_VELOCITY) {
    int vel_pid =  pid(dcm_target_velocity, dcm_fb_velocity, 
                    VEL_P, VEL_I, VEL_D, 
                    VEL_I_CLAMP, VEL_OUT_CLAMP, VEL_PUNCH, VEL_DEADZONE, 
                    &vel_last, &vel_accumulate);
                    
    dcm_velocity_power += vel_pid;
    limit(&dcm_velocity_power, -PWM_MAX, PWM_MAX);
    if (vel_pid > 0) {
      analogWrite(PIN_MOTOR_IN_1, vel_pid);
      digitalWrite(PIN_MOTOR_IN_2, LOW);
    } else if (vel_pid < 0) {
      digitalWrite(PIN_MOTOR_IN_1, LOW);
      analogWrite(PIN_MOTOR_IN_2, abs(vel_pid));
    }
  }
}

// Updates the position feedback and direction based on the motor encoder values
void update_dcm_pos_feedback() {
  // Get new encoder values
  int a = digitalRead(PIN_MOTOR_ENCODER_A);
  int b = digitalRead(PIN_MOTOR_ENCODER_B);
  int pos_offset = 0;

  // Determine state change
  // Direction and states changed
  if (dcm_fb_a == LOW && dcm_fb_b == LOW) {
    if (a == LOW  && b == HIGH) { // Rotate CC
      pos_offset = 1*DCM_CC;
      dcm_fb_direction = DCM_CC;
    } else if (a == HIGH && b == HIGH) { // Skipped state
      pos_offset = 2*dcm_fb_direction;
    } else if (a == HIGH && b == LOW)  { // Rotate CW
      pos_offset = 1*DCM_CCW;
      dcm_fb_direction = DCM_CCW;
    } // a == LOW and b == LOW is the same state
    
  } else if (dcm_fb_a == LOW && dcm_fb_b == HIGH) {
    if (a == HIGH  && b == HIGH) { // Rotate CC
      pos_offset = 1*DCM_CC;
      dcm_fb_direction = DCM_CC;
    } else if (a == HIGH && b == LOW) { // Skipped state
      pos_offset = 2*dcm_fb_direction;
    } else if (a == LOW && b == LOW)  { // Rotate CW
      pos_offset = 1*DCM_CCW;
      dcm_fb_direction = DCM_CCW;
    } // a == LOW and b == HIGH is the same state
    
  } else if (dcm_fb_a == HIGH && dcm_fb_b == HIGH) {
    if (a == HIGH  && b == LOW) { // Rotate CC
      pos_offset = 1*DCM_CC;
      dcm_fb_direction = DCM_CC;
    } else if (a == LOW && b == LOW) { // Skipped state
      pos_offset = 2*dcm_fb_direction;
    } else if (a == LOW && b == HIGH)  { // Rotate CW
      pos_offset = 1*DCM_CCW;
      dcm_fb_direction = DCM_CCW;
    } // a == HIGH and b == HIGH is the same state
    
  } else if (dcm_fb_a == HIGH && dcm_fb_b == LOW) {
    if (a == LOW && b == LOW) { // Rotate CC
      pos_offset = 1*DCM_CC;
      dcm_fb_direction = DCM_CC;
    } else if (a == LOW && b == HIGH) { // Skipped state
      pos_offset = 2*dcm_fb_direction;
    } else if (a == HIGH && b == HIGH)  { // Rotate CW
      pos_offset = 1*DCM_CCW;
      dcm_fb_direction = DCM_CCW;
    } // a == HIGH and b == HIGH is the same state
  }

  // Update global variables
  dcm_fb_position_ticks += pos_offset;
  dcm_fb_position = (dcm_fb_position_ticks * DCM_TICKS_TO_DEG_NUMERATOR) / DCM_TICKS_TO_DEG_DENOMINATOR;
}

<<<<<<< HEAD
void servo_goto(int deg)
{
  int pulselen = map(deg, 0, 180, SERVOMIN, SERVOMAX);
  uint8_t servonum = 15; // address on the board where the servo is wired
  servo.setPWM(servonum, 0, pulselen);
=======
/*
 * Params
 *  Target:     The target value
 *  Current:    T
 *  P:          P term
 *  I:          I term
 *  D:          D term
 *  i_clamp:    Limits the maximum and minimum absolute values of *accumulate error. 
 *              Prevents integral windup
 *  out_clamp:  Maximum absolute value of the returned value
 *  punch:      Amount to add to absulute value of output.
 *              Helps overcome stiction
 *  deadzone:   Sets output to zero if absolute value of error is less than this
 *  *last:      Previous value of current. This function updates last
 *  *accumulate:  Integral windup term.
 */
int pid(int target, int current, int p, int i, int d, int i_clamp, int out_clamp, int punch, int deadzone, int *last, int *accumulate_error) {
  int error = target - current;

  // Deadzone stuff
  if (-deadzone < error && error < deadzone) {
    return 0;
  }

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
  if (error > 0) {
    punch_term = punch;
  } else if (error < 0) {
    punch_term = -punch;
  }

  int sum = p_term + i_term + d_term + punch_term;
  limit(&sum, -out_clamp, out_clamp);
//  Serial.print("T: ");
//  Serial.print(target);
//  Serial.print(" \t C: ");
//  Serial.println(current);
//  Serial.print(" \t E: ");
//  Serial.print(error);
//  Serial.print("     \t S: ");
//  Serial.println(sum);
  return sum;
>>>>>>> 811eabd2c6cb3c8ab2f44f67c82fd57c6dbfc73c
}
