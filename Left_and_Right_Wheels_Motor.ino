/* Pragash Mohanarajah (C) November 2021. */

/*
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2
---->  http://www.adafruit.com/products/1438
*/

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
unsigned long previousMillis = 0;
long duration;
const int ledPin = 8, echoPin = 2, trigPin = 3;
int distance = 0, ledState = LOW;

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
  // Configure LED pin as an output
  pinMode(ledPin, OUTPUT);
  // Configure Trigger pin of HC-SR04 as an output
  pinMode(trigPin, OUTPUT);
  // Configure Echo pin of HC-SR04 as an input
  pinMode(echoPin, INPUT);
  // Initialise all output pins as LOW
  digitalWrite(ledPin, LOW);
  digitalWrite(trigPin, LOW);
}

void loop() {
  unsigned long currentMillis = millis();
  unsigned long time_elapsed = currentMillis - previousMillis;
  if ((time_elapsed <= 5) && (distance == 0)) {
    digitalWrite(trigPin, HIGH);
    delay(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = duration * 3.4 / 20;
    if (distance < 150) decel = true;
  }
  if (time_elapsed >= 250) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
    distance = 0;
  }
  if ((time_elapsed % 10) == 0) {
    if (accel) {
      leftMotor->run(FORWARD);
      rightMotor->run(FORWARD);
      for (uint8_t i=0; i<255; i++) {
        leftMotor->setSpeed(i);
        rightMotor->setSpeed(i);
      }
      accel = !accel;
    }
    if (decel) {
      leftMotor->run(FORWARD);
      rightMotor->run(FORWARD);
      for (uint8_t i=0; i<255; i--) {
        leftMotor->setSpeed(i);
        rightMotor->setSpeed(i);
      }
      decel = !decel;
    }
  }
}
