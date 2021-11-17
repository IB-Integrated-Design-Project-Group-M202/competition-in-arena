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
unsigned long amberLED_Millis = 0; currentMillis = 0; time_elapsed = 0; trigger_Millis = 0; time_trigger = 0;
const int amberLED_Pin = 8, echoPin = 2, trigPin = 3;
int distance = 0, ledState = LOW;
uint8_t speed = 0; amberLED_duration = 250; trigger_duration = 10;

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
  pinMode(amberLED_Pin, OUTPUT);
  // Configure Trigger pin of HC-SR04 as an output
  pinMode(trigPin, OUTPUT);
  // Configure Echo pin of HC-SR04 as an input
  pinMode(echoPin, INPUT);
  // Initialise all output pins as LOW
  digitalWrite(ledPin, LOW);
  digitalWrite(trigPin, LOW);
}

void loop() {
  currentMillis = millis();
  time_elapsed = currentMillis - amberLED_Millis;
  time_trigger = currentMillis - triger_Millis;
  if (distance == 0) {
    if (time_elapsed == 2) {
      digitalWrite(trigPin, HIGH);
      trigMillis = currentMillis;
    if (time_trigger >= trigger_duration) {
      digitalWrite(trigPin, LOW);
      duration = pulseIn(echoPin, HIGH);
      distance = duration * 3.4 / 20;
      if (distance < 150) decel = true;
    }
  }
  if (time_elapsed >= amberLED_duration) {
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
      if (speed == 254) {
        accel = !accel;
      }
      leftMotor->run(FORWARD);
      rightMotor->run(FORWARD);
      speed += 1;
      leftMotor->setSpeed(speed);
      rightMotor->setSpeed(speed);
    }
    if (decel) {
      if (speed == 1) {
        decel = !decel;
      }
      leftMotor->run(FORWARD);
      rightMotor->run(FORWARD);
      speed -= 1;
      leftMotor->setSpeed(speed);
      rightMotor->setSpeed(speed);
    }
  }
}
