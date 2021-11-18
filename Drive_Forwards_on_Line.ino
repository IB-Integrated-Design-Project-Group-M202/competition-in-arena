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
unsigned long amberLED_Millis = 0, currentMillis = 0, time_elapsed = 0, echo_duration = 0;
const int echoPin = 2, trigPin = 3, leftLineSensor = 4, rightLineSensor = 7, amberLED_Pin = 8, numReadings = 576, pt1_Pin = A1, pt2_Pin = A2;
int distance = 0, amberLED_State = LOW, leftSensorStatus = LOW, rightSensorStatus = LOW, readings[numReadings], readIndex = 0, total = 0, average = 0, pt_Min = 1023, pt_Max = 0;
uint8_t leftSpeed = 0, rightSpeed = 0, amberLED_duration = 250, trigger_duration = 10;

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
  // Initialize all the phototranistor readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
  // Configure LED pin as an output
  pinMode(amberLED_Pin, OUTPUT);
  // Configure Trigger pin of HC-SR04 as an output
  pinMode(trigPin, OUTPUT);
  // Configure Echo pin of HC-SR04 as an input
  pinMode(echoPin, INPUT);
  // Configure Phototransistors as inputs
  pinMode(pt1_Pin, INPUT);
  pinMode(pt2_Pin, INPUT);
  // Initialise all output pins as LOW
  digitalWrite(amberLED_Pin, LOW);
  digitalWrite(trigPin, LOW);
}

void loop() {
  currentMillis = millis();
  time_elapsed = currentMillis - amberLED_Millis;
  leftSensorStatus = digitalRead(leftLineSensor);
  rightSensorStatus = digitalRead(rightLineSensor);
  
  total = total - readings[readIndex];
  readings[readIndex] = (analogRead(pt1_Pin) + analogRead(pt2_Pin)) / 2;
  if (readings[readIndex] < pt_Min && readings[readIndex] > 20) pt_Min = readings[readIndex];
  if (readings[readIndex] > pt_Max && readings[readIndex] < 1000) pt_Max = readings[readIndex];
  total = total + readings[readIndex];
  readIndex = readIndex + 1;
  if (readIndex >= numReadings) {
    readIndex = 0;
    average = total / numReadings;
    Serial.println(average);
    if (pt_Max > 800) leftMotor->run(RELEASE); rightMotor->run(RELEASE);
    delay(12);
  }
  
  if (time_elapsed >= amberLED_duration) {
    // save the last time you blinked the LED
    amberLED_Millis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (amberLED_State == LOW) amberLED_State = HIGH; else amberLED_State = LOW;

    // set the LED with the ledState of the variable:
    digitalWrite(amberLED_Pin, amberLED_State);
    
    digitalWrite(trigPin, HIGH);
    delay(trigger_duration);
    digitalWrite(trigPin, LOW);
    echo_duration = pulseIn(echoPin, HIGH);
    distance = echo_duration * 3.4 / 20;
    if (distance < 150) decel = true;
  }
  if (leftSensorStatus == LOW && rightSensorStatus == LOW) {
    if (leftSpeed != rightSpeed) {
      leftSpeed = (leftSpeed + rightSpeed) / 2;
      rightSpeed = (leftSpeed + rightSpeed) / 2;
    } else
    if (leftSpeed < 255 || rightSpeed < 255) if (!decel) accel = true;
  } else
  if (leftSensorStatus == HIGH && rightSensorStatus == LOW) {
    leftSpeed += 5; rightSpeed -= 5;
  } else
  if (leftSensorStatus == LOW && rightSensorStatus == HIGH) {
    leftSpeed -= 5; rightSpeed += 5;
  }
  if ((time_elapsed % 10) <= 2) {
    if (accel) {
      if (leftSpeed < 255) leftSpeed += 1;
      if (rightSpeed < 255) rightSpeed += 1;
      if (leftSpeed == 255 && rightSpeed == 255) accel = !accel;
      leftMotor->run(FORWARD);
      rightMotor->run(FORWARD);
      leftMotor->setSpeed(leftSpeed);
      rightMotor->setSpeed(rightSpeed);
    }
    if (decel) {
      if (leftSpeed > 0) leftSpeed -= 1;
      if (rightSpeed > 0) rightSpeed -= 1;
      if (leftSpeed == 0 && rightSpeed == 0) decel = !decel;
      leftMotor->run(FORWARD);
      rightMotor->run(FORWARD);
      leftMotor->setSpeed(leftSpeed);
      rightMotor->setSpeed(rightSpeed);
    }
  }
}
