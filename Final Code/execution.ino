#include "variables.h"
#include "functions.h"

void setup() {

  // Configure the motor shield
  while (!AFMS.begin()) { // Check whether the motor shield is properly connected
    digitalWrite(redLED_Pin, HIGH);
  }
  
  // Configure IMU
  while (!IMU.begin()) { // Check whether the IMU works
    digitalWrite(amberLED_Pin, HIGH);
  }
  reset_gyroscope();
  
  // Configure Left and Right Motors
  leftMotor->setSpeed(150);
  rightMotor->setSpeed(150);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  leftSpeed = 255;
  rightSpeed = 255;

  // Configure the distance sensor
  HCSR04.begin(trigPin, echoPin);
  digitalWrite(trigPin, LOW);

  // Configure line sensors
  pinMode(leftLineSensor, INPUT);
  pinMode(rightLineSensor, INPUT);
  
  // Configure IR sensors
  pinMode(irr_Pin, INPUT);
  pinMode(pt1_Pin, INPUT);
  pinMode(pt2_Pin, INPUT);
  for (int i = 0; i < a_size; i++){
    s1m1sa[i]=0; s2m1sa[i]=0;
  }
  
  // Configure LEDs
  pinMode(amberLED_Pin, OUTPUT);
  pinMode(redLED_Pin, OUTPUT);
  pinMode(greenLED_Pin, OUTPUT);
  digitalWrite(amberLED_Pin, LOW);
  digitalWrite(redLED_Pin, LOW);
  digitalWrite(greenLED_Pin, LOW);

  // Start Time
  start_time_m = millis();

}

void loop() {
  check_timeout();
  if (!gyro_calibrated) reset_gyroscope();
  measure_gyroscope();
  if (gyro_calibrated && !search_area) drive_on_line(); else
  if (search_area && (!stopped && !arrived)) drive_on_line_to_obstruction(); else
  if (stopped && !identifiedLine) identify_dummy(); else
  if (identifiedLine && !in_range) search_and_align_and_identify(); else
  if (in_range && !timeout) {
    if (dummy_angle_3 == 0 && identifiedArea) escape(); else
    if (dummy_angle_3 != 0 && (!aligned && !on_line)) align_to_line();
    if (aligned && (!arrived && !on_line)) drive_to_line(); else
    if (arrived && on_line) {
      if (!aligned) align_with_line(); else
      if (aligned && search_area) drive_on_line(); else
      if (aligned && !search_area) drive_on_line_to_obstruction();
    }
  }
}
