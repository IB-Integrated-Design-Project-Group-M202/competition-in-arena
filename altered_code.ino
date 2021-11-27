#include "variables.h"
#include "functions.h"

//<-------------------------------------------------------------------------------------------------------------SETUP BEGINS
void setup() {

  Serial.begin(9600);
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

  // Configure the distance sensor
  HCSR04.begin(trigPin, echoPin);
  digitalWrite(trigPin, LOW);

  // Configure line sensors
  pinMode(leftLineSensor, INPUT);
  pinMode(rightLineSensor, INPUT);
  
  // Configure IR sensors
  pinMode(pt1_Pin, INPUT);
  pinMode(pt2_Pin, INPUT);
  for (int i = 0; i < a_size; i++){
    s1m1sa[i]=0;
    s2m1sa[i]=0;
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
  leftSpeed = 255; rightSpeed = 255;

}

//<---------------------------------------------------------------------------------------------------------------LOOP BEGINS
void loop() {  
  // Updates all variables
  check_timeout();
  if (!gyro_calibrated) reset_gyroscope();
  measure_gyroscope();
  if (gyro_calibrated && !search_area) drive_on_line(); else
  if (search_area && !stopped)         drive_on_line_to_dummy(); else
  if (stopped && !identifiedLine)      identify_dummy(); else
  if (identifiedLine && !aligned)      align_with_dummy(); else
  if (aligned && !arrived)             drive_to_dummy(); else
  if (arrived && !identifiedArea)      identify_dummy(); else
  if (identifiedArea && !timeout) { aligned = false; arrived = false; identifiedArea = false; }
  
}
