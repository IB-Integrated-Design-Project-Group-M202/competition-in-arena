/* Pragash Mohanarajah (C) November 2021. */

#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select and configure port M1
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
// Select and configure port M2
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

const int r_Pin = A0, pt1_Pin = A1, pt2_Pin = A2, greenLED_Pin = 12, redLED_Pin = 13;

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
  
  // Configure two phototransistors as analog inputs
  pinMode(pt1_Pin, INPUT);
  pinMode(pt2_Pin, INPUT);
  // Configure IR receiver as an analog input_pullup, i.e. with inverted logic with respect to 5V V_CC.
  pinMode(r_Pin, INPUT_PULLUP);
  // Configure green and red indication LEDs as analog outputs
  pinMode(greenLED_Pin, OUTPUT);
  pinMode(redLED_Pin, OUTPUT);
}

void loop() {
  // Turn robot right slowly
  leftMotor->setSpeed(50);
  rightMotor->setSpeed(50);
  leftMotor->run(FORWARD);
  rightMotor->run(BACKWARD);
}
