/* Name: Jamie Lau */
/* Course: TEJ4M  */
/*
 Program Purpose: 
 The purpose of this program is to use the accelerometer, that is inside a phone, as a remote control in order to control a car. The data from the phone
 is transferred through the OneSheeld and the data from the accelerometer is used to control the direction in which the car will move.
*/

/*DEFINING THE MOTOR*/
#include <Servo.h>
#define MOTORLATCH 12
#define MOTORCLK 4
#define MOTORENABLE 7
#define MOTORDATA 8
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR3_A 5
#define MOTOR3_B 7
#define MOTOR4_A 0
#define MOTOR4_B 6
#define MOTOR1_PWM 11
#define MOTOR2_PWM 3
#define MOTOR3_PWM 6
#define MOTOR4_PWM 5
#define SERVO1_PWM 10
#define SERVO2_PWM 9
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4
 
Servo servo_1;
Servo servo_2;

//OneSheeld declarations

#define CUSTOM_SETTINGS
#define INCLUDE_ACCELEROMETER_SENSOR_SHIELD

#include <OneSheeld.h>

//variables to store the x, y, and z values from the accelerometer
float y, z;

int maxSpeed = 255;                         // the max speed at which the motors can rotate

//booleans for the movement of the car
boolean forward = false;                    // determines if car is moving forward or backward
boolean moving = false;                     // determines if car is going to move
boolean forwardold = forward;               // used to determine if the motors should release

void setup() {
  // serial input for motor
  Serial.begin(9600);
  Serial.println("Simple Adafruit Motor Shield sketch");
  servo_1.attach(SERVO1_PWM);
  servo_2.attach(SERVO2_PWM);
  // setup for OneSheeld
  OneSheeld.begin ();
}

void loop() {
  //Get the values of the accelerometer in the 3 dimensions
  y=AccelerometerSensor.getY();
  z=AccelerometerSensor.getZ();

  /*MOVEMENT CONTROLS*/
  if (z < 0.2) {                            // checks if the phone is tilted backwards. If it is, the car will move in reverse.
    if (y < -1) {                           // if the phone is tilted to the left, the car will turn right
      int q = maxSpeed / 7;                       // variable to store the increments at which the car will move when the phone is turned
      int tSpeed = maxSpeed + floor (y) * q;      // turn speed based on how much the phone is turned
      if (tSpeed < 0) {
        tSpeed = 0;                         // turn speed is regulated, so it ccannot be less than 0
      }
      motor (3, BACKWARD, maxSpeed);
      motor (4, BACKWARD, tSpeed);
      delay (10);                           // stability delay
    } else if (y > 1) {                     // if the phone is tilted to the right, the car will turn right
      int q = maxSpeed / 7;                      // variable to store the increments at which the car will move when the phone is turned
      int tSpeed = maxSoeed - floor (y) * q;     // turn speed based on how much the phone is turned
      if (tSpeed < 0) {
        tSpeed = 0;                         // turn speed is regulated, so it cannot be less than 0
      }
      motor (3, BACKWARD, tSpeed);
      motor (4, BACKWARD, maxSpeed);
      delay (10);                           // stability delay
    } else {                                // if phone is not tilted to the side, the car will reverse at full speed
      motor (3, BACKWARD, maxSpeed);
      motor (4, BACKWARD, maxSpeed);
      delay (10);                           // stability delay
    }
  } else if (z > 4) {                       // checks if the phone is tilted forwards. If it is, the car will move forward.
    if (y < -1) {                           // if the phone is tilted to the left, the car will turn left
      int q = maxSpeed / 7;                      // variable to store the increments at which the car will move when the phone is turned
      int tSpeed = maxSpeed + floor (y) * q;     // turn speed based on how much the phone is turned
      if (tSpeed < 0) {
        tSpeed = 0;                         // speed is regulated, so it cannot be less than 0
      }
      motor (3, FORWARD, maxSpeed);
      motor (4, FORWARD, tSpeed);
      delay (10);                           // stability delay
    } else if (y > 1) {                     // if the phone is tilted to the right, the car will turn right
      int q = maxSpeed / 7;                      // variable to store the increments at which the car will move when the phone is turned
      int tSpeed = maxSpeed - floor (y) * q;     // turn speed based on how much the phone is turned
      if (tSpeed < 0) {
        tSpeed = 0;                         // speed is regulated, so it cannot be less than 0
      }
      motor (3, FORWARD, tSpeed);
      motor (4, FORWARD, maxSpeed);
      delay (10);                           // stability delay
    } else {                                // if phone is not tilted to the side, car will go straight at full speed
        motor (3, FORWARD, maxSpeed);
        motor (4, FORWARD, maxSpeed);
        delay (10);                         // stability delay
    }
  } else {                                  //if phone is upright, the car stops/does not move
    motor (3, BRAKE, 0);
    motor (4, BRAKE, 0);
    delay (10);                             // stability delay
  } 
}

//Motor commands listed below

void motor(int nMotor, int command, int speed)
{
  int motorA, motorB;
  if (nMotor >= 1 && nMotor <= 4)
  {
    switch (nMotor)
    {
      case 1:
        motorA = MOTOR1_A;
        motorB = MOTOR1_B;
        break;
      case 2:
        motorA = MOTOR2_A;
        motorB = MOTOR2_B;
        break;
      case 3:
        motorA = MOTOR3_A;
        motorB = MOTOR3_B;
        break;
      case 4:
        motorA = MOTOR4_A;
        motorB = MOTOR4_B;
        break;
      default:
        break;
    }
    switch (command)
    {
      case FORWARD:
        motor_output (motorA, HIGH, speed);
        motor_output (motorB, LOW, -1); // -1: no PWM set
        break;
      case BACKWARD:
        motor_output (motorA, LOW, speed);
        motor_output (motorB, HIGH, -1); // -1: no PWM set
        break;
      case BRAKE:
        motor_output (motorA, LOW, 255); // 255: fully on.
        motor_output (motorB, LOW, -1); // -1: no PWM set
        break;
      case RELEASE:
        motor_output (motorA, LOW, 0); // 0: output floating.
        motor_output (motorB, LOW, -1); // -1: no PWM set
        break;
      default:
        break;
    }
  }
}
void motor_output (int output, int high_low, int speed)
{
  int motorPWM;
  switch (output)
  {
    case MOTOR1_A:
    case MOTOR1_B:
      motorPWM = MOTOR1_PWM;
      break;
    case MOTOR2_A:
    case MOTOR2_B:
      motorPWM = MOTOR2_PWM;
      break;
    case MOTOR3_A:
    case MOTOR3_B:
      motorPWM = MOTOR3_PWM;
      break;
    case MOTOR4_A:
    case MOTOR4_B:
      motorPWM = MOTOR4_PWM;
      break;
    default:
      speed = -3333;
      break;
  }
  
  if (speed != -3333)
  {
    shiftWrite(output, high_low);
    // set PWM only if it is valid
    if (speed >= 0 && speed <= 255)
    {
      analogWrite(motorPWM, speed);
    }
  }
}

void shiftWrite(int output, int high_low)
{
  static int latch_copy;
  static int shift_register_initialized = false;
  // Do the initialization on the fly,
  // at the first time it is used.
  if (!shift_register_initialized)
  {
    // Set pins for shift register to output
    pinMode(MOTORLATCH, OUTPUT);
    pinMode(MOTORENABLE, OUTPUT);
    pinMode(MOTORDATA, OUTPUT);
    pinMode(MOTORCLK, OUTPUT);

    // Set pins for shift register to default value (low);
    digitalWrite(MOTORDATA, LOW);
    digitalWrite(MOTORLATCH, LOW);
    digitalWrite(MOTORCLK, LOW);

    // Enable the shift register, set Enable pin Low.
    digitalWrite(MOTORENABLE, LOW);

    // start with all outputs (of the shift register) low
    latch_copy = 0;
    shift_register_initialized = true;
  }
  // The defines HIGH and LOW are 1 and 0.
  // So this is valid.
  bitWrite(latch_copy, output, high_low);
  shiftOut(MOTORDATA, MOTORCLK, MSBFIRST, latch_copy);
  delayMicroseconds(5); // For safety, not really needed.
  digitalWrite(MOTORLATCH, HIGH);
  delayMicroseconds(5); // For safety, not really needed.
  digitalWrite(MOTORLATCH, LOW);
}
