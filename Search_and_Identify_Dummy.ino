/* Pragash Mohanarajah (C) November 2021. */

#include <Adafruit_MotorShield.h>
#include <Arduino_LSM6DS3.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select and configure port M1
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
// Select and configure port M2
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

bool gyro_calibrated = false, search_area = false, pt1_maximum = false, pt2_maximum = false, aligned = false, arrived = false, identified = false;
unsigned long startMillis = 0, currentMillis = 0, elapsedMillis = 0, amberLED_Millis = 0, last_gyro = 0, last_amber = 0;
const int echoPin = 2, trigPin = 3, r_Pin = A0, pt1_Pin = A1, pt2_Pin = A2, amberLED_Pin = 8, greenLED_Pin = 12, redLED_Pin = 13, indicatorDelay = 5000, numReadings = 150;
int pt1_readings[numReadings], pt2_readings[numReadings], readIndex = 0, pt1_total = 0, pt2_total = 0, pt1_average = 0, pt2_average = 0, pt1_Min = 1023, pt2_Min = 1023, pt1_Max = 0, pt2_Max = 0;
int amberLED_State = LOW, distance, gyroMillis = 0;
float x, y, z, angle_total = 0, angle_offset = 0, angle_turned = 0, pt1_angle = 0, pt2_angle = 0, dummy_angle = 0;
uint8_t dummy = 0, echo_duration = 0, amberLED_duration = 250, trigger_duration = 10;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  while (!Serial);

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    Serial.println(F("Could not find Motor Shield. Check wiring."));
    while (1);
  }
  Serial.println(F("Motor Shield found."));
  
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.println(F("IMU initialized."));

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
    pt1_readings[thisReading] = 0;
    pt2_readings[thisReading] = 0;
  }
  // Configure two phototransistors as analog inputs
  pinMode(pt1_Pin, INPUT);
  pinMode(pt2_Pin, INPUT);
  // Configure IR receiver as an analog input_pullup, i.e. with inverted logic with respect to 5V V_CC.
  pinMode(r_Pin, INPUT_PULLUP);
  // Configure Trigger pin of HC-SR04 as an output
  pinMode(trigPin, OUTPUT);
  // Configure Echo pin of HC-SR04 as an input
  pinMode(echoPin, INPUT);
  // Configure green and red indication LEDs as analog outputs
  pinMode(greenLED_Pin, OUTPUT);
  pinMode(redLED_Pin, OUTPUT);
  // Initialise all outputs as LOW
  digitalWrite(greenLED_Pin, LOW);
  digitalWrite(redLED_Pin, LOW);
  
  startMillis = millis();
}

void calibrate_gyro() {
  int readings;
  while (readings < 20000) {
    if (IMU.gyroscopeAvailable()) IMU.readGyroscope(x, y, z);
    angle_total += z; readings += 1; angle_offset = angle_total / readings;
  }
  gyro_calibrated = true;
}

void pt_maxima() {
  pt1_readings[readIndex] = analogRead(pt1_Pin); pt2_readings[readIndex] = analogRead(pt2_Pin);
  if (pt1_readings[readIndex] > pt1_Max) pt1_Max = pt1_readings[readIndex];
  else { if (pt1_readings[readIndex] < pt1_readings[readIndex - 2]) pt1_maximum = true; }
  if (pt2_readings[readIndex] > pt2_Max) pt2_Max = pt2_readings[readIndex];
  else { if (pt2_readings[readIndex] < pt2_readings[readIndex - 2]) pt2_maximum = true; }
  if (pt1_maximum) pt1_angle = angle_turned;
  if (pt2_maximum) pt2_angle = angle_turned;
  readIndex += 1;
  if (readIndex >= numReadings) readIndex = 0;
}

void pt_average() {
  pt1_total = pt1_total - pt1_readings[readIndex]; pt2_total = pt2_total - pt2_readings[readIndex];
  pt1_readings[readIndex] = analogRead(pt1_Pin); pt2_readings[readIndex] = analogRead(pt2_Pin);
  pt1_total = pt1_total + pt1_readings[readIndex]; pt2_total = pt2_total + pt2_readings[readIndex];
  readIndex += 1;
  if (readIndex >= numReadings) {
    readIndex = 0;
    pt1_average = pt1_total / numReadings; pt2_average = pt2_total / numReadings;
    delay(12);
  }
}

void align_with_dummy() {
  // Turn robot right slowly
  leftMotor->setSpeed(60);
  rightMotor->setSpeed(60);
  if (dummy_angle == 0) {
    leftMotor->run(BACKWARD); rightMotor->run(FORWARD);
  } else {
    if (abs(angle_turned - dummy_angle) <= 0.5) {
      leftMotor->run(RELEASE); rightMotor->run(RELEASE); aligned = true;
    } else { leftMotor->run(FORWARD); rightMotor->run(BACKWARD); }
  }
  if (pt1_angle == 0 || pt2_angle == 0) pt_maxima(); else dummy_angle = (pt1_angle + pt2_angle) / 2;
}

void drive_to_dummy() {
  if (amberLED_Millis >= amberLED_duration) {
    // save the last time you blinked the LED
    last_amber = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (amberLED_State == LOW) amberLED_State = HIGH; else amberLED_State = LOW;

    // set the LED with the ledState of the variable:
    digitalWrite(amberLED_Pin, amberLED_State);
    
    if (distance < 150 && search_area) { arrived = true; leftMotor->run(RELEASE); rightMotor->run(RELEASE); }
    else { leftMotor->setSpeed(255); rightMotor->setSpeed(255); leftMotor->run(FORWARD); rightMotor->run(FORWARD); }
  }
}

void identify_dummy() {
  
}

void loop() {
  currentMillis = millis();
  elapsedMillis = currentMillis - startMillis;
  gyroMillis = currentMillis - last_gyro;
  amberLED_Millis = currentMillis - last_amber;
  if (amberLED_Millis >= amberLED_duration) {
    digitalWrite(trigPin, HIGH);
    delay(trigger_duration);
    digitalWrite(trigPin, LOW);
    echo_duration = pulseIn(echoPin, HIGH);
    distance = echo_duration * 3.4 / 20;
  }
  if (IMU.accelerationAvailable()) IMU.readAcceleration(x, y, z);
  if (y > 0.20) search_area = true;
  if (IMU.gyroscopeAvailable()) IMU.readGyroscope(x, y, z);
  if (gyroMillis >= 50) angle_turned += (z - angle_offset) * gyroMillis/1E3 * 180/160.7;
  last_gyro = millis();
  if (!gyro_calibrated && !aligned) calibrate_gyro();
  if (gyro_calibrated && !aligned) align_with_dummy();
  if (gyro_calibrated && aligned && !arrived) drive_to_dummy();
  if (gyro_calibrated && aligned && arrived && !identified) identify_dummy();
  switch (dummy) {
    case 0:
      break;
    case 1:
      digitalWrite(greenLED_Pin, HIGH);
      digitalWrite(redLED_Pin, HIGH);
      delay(indicatorDelay);
      digitalWrite(greenLED_Pin, LOW);
      digitalWrite(redLED_Pin, LOW);
      break;
    case 2:
      digitalWrite(redLED_Pin, HIGH);
      delay(indicatorDelay);
      digitalWrite(redLED_Pin, LOW);
      break;
    case 3:
      digitalWrite(greenLED_Pin, HIGH);
      delay(indicatorDelay);
      digitalWrite(greenLED_Pin, LOW);
      break;
  }
}
