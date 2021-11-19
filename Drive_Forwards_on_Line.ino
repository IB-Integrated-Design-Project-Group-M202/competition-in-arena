/* Pragash Mohanarajah (C) November 2021. */

#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select and configure port M1
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
// Select and configure port M2
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

// Define System parameters


// Create Global variables
bool accel = true, decel = false;
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
  
}

void loop() {
  leftSensorStatus = digitalRead(leftLineSensor);
  rightSensorStatus = digitalRead(rightLineSensor);
  leftMotor->setSpeed(leftSpeed);
  rightMotor->setSpeed(rightSpeed);
  
  if (leftSensorStatus == LOW && rightSensorStatus == LOW) {
    if (leftSpeed != rightSpeed) {
      leftSpeed = (leftSpeed + rightSpeed) / 2;
      rightSpeed = (leftSpeed + rightSpeed) / 2;
    } else
    if (leftSpeed == rightSpeed) if (leftSpeed < 255 || rightSpeed < 255) if (!decel) accel = true;
  } else
  if (leftSensorStatus == HIGH && rightSensorStatus == LOW) {
    leftSpeed += 5; rightSpeed -= 5;
  } else
  if (leftSensorStatus == LOW && rightSensorStatus == HIGH) {
    leftSpeed -= 5; rightSpeed += 5;
  }
  if (accel) {
    if (leftSpeed < 255) leftSpeed += 1;
    if (rightSpeed < 255) rightSpeed += 1;
    if (leftSpeed == 255 && rightSpeed == 255) accel = !accel;
    leftMotor->run(FORWARD);
    rightMotor->run(FORWARD);
    delay(10);
  }
  if (decel) {
    if (leftSpeed > 0) leftSpeed -= 1;
    if (rightSpeed > 0) rightSpeed -= 1;
    if (leftSpeed == 0 && rightSpeed == 0) decel = !decel;
    leftMotor->run(FORWARD);
    rightMotor->run(FORWARD);
    delay(10);
  }
}
