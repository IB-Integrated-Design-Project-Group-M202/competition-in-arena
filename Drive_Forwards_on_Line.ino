/* Pragash Mohanarajah (C) November 2021. */
// IMPORTANT NOTICE
// This file has been abandoned, due to irreliability of Digital Line Sensor Control
// Line_Sensors_Analog_PID_Motor_Control.ino uses Analog Line Sensor Control coupled with PID Control
// Refer to this file for Drive Forwards on Line Algorithm implementation

#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select and configure port M1
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
// Select and configure port M2
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

// Define System parameters


// Create Global variables
bool accel = true, decel = false, on_line = true;
const int leftLineSensor = 4, rightLineSensor = 7;
int leftSensorStatus = LOW, rightSensorStatus = LOW;
uint8_t leftSpeed = 0, rightSpeed = 0;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println(F("Adafruit Motor Shield v2.3 Test for DC Motors M1 & M2"));

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    Serial.println(F("Could not find Motor Shield. Check wiring."));
    while (1);
  }
  Serial.println(F("Motor Shield found."));

  // Set a speed of 150 to start, where 0 is off & 255 is max speed
  leftMotor->setSpeed(150);
  rightMotor->setSpeed(150);
  // Turn on motors
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  // Turn off motors
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  
  // Configure Left and Right Line Sensors as inputs
  pinMode(leftLineSensor, INPUT);
  pinMode(rightLineSensor, INPUT);
}

void loop() {
  leftSensorStatus = digitalRead(leftLineSensor);
  rightSensorStatus = digitalRead(rightLineSensor);
  leftMotor->setSpeed(leftSpeed);
  rightMotor->setSpeed(rightSpeed);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  
  if (leftSensorStatus == LOW && rightSensorStatus == LOW) {
    on_line = true;
    if (leftSpeed != rightSpeed) {
      leftSpeed = (leftSpeed + rightSpeed) / 2;
      rightSpeed = (leftSpeed + rightSpeed) / 2;
    } else
    if (leftSpeed == rightSpeed && !decel) { leftSpeed = 255; rightSpeed = 255; }
  } else
  if (leftSensorStatus == HIGH || rightSensorStatus == HIGH) on_line = false;
  if (!on_line && leftSensorStatus == HIGH && rightSensorStatus == LOW) {
    leftSpeed -= 51; leftMotor->setSpeed(leftSpeed); rightMotor->setSpeed(rightSpeed); delay(50);
  } else
  if (!on_line && leftSensorStatus == LOW && rightSensorStatus == HIGH) {
    rigbtSpeed -= 51; leftMotor->setSpeed(leftSpeed); rightMotor->setSpeed(rightSpeed); delay(50);
  }
  if (accel) {
    if (leftSpeed < 255) leftSpeed += 51;
    if (rightSpeed < 255) rightSpeed += 51;
    if (leftSpeed == 255 && rightSpeed == 255) accel = false;
  }
  if (decel) {
    if (leftSpeed > 0) leftSpeed -= 51;
    if (rightSpeed > 0) rightSpeed -= 51;
    if (leftSpeed == 0 && rightSpeed == 0) decel = false;
  }
}
