#include <Adafruit_MotorShield.h>
#include <Arduino_LSM6DS3.h>
#include <HCSR04.h>

// Global variables for state of the robot
unsigned long current_time_u, current_time_m, start_time_m, finish_time_m = 3E5;
bool accel = true, decel = false, timeout = false;
bool on_line = true, on_ramp = false, search_area = false;
bool dummy_reached = false, aligned = false, arrived = false, identified = false;

// Global variables and definitions for motors
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the motor shield object with the default I2C address
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1); // Select and configure port M1
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2); // Select and configure port M2
uint8_t leftSpeed = 0, rightSpeed = 0, leftDirection = FORWARD, rightDirection = FORWARD, trigger_duration = 10;

// Global variables and definitions for distance sensor
#define echoPin 2
#define trigPin 3
double* distances;

// Global variables and definitions for line sensors
#define leftLineSensor 4
#define rightLineSensor 7
int leftSensorStatus = LOW, rightSensorStatus = LOW;

// Global variables and definitions for IR sensors
#define irr_Pin A0
#define pt1_Pin A1
#define pt2_Pin A2
const int numReadings = 576;
int readings[numReadings];
int readIndex = 0, total = 0, average = 0, pt_Min = 1023, pt_Max = 0;
bool pt1_maximum = false, pt2_maximum = false;

// Global variables and definitions for LEDs
#define amberLED_Pin 8
#define greenLED_Pin 12
#define redLED_Pin 13
uint8_t amberLED_duration = 250;
int amberLED_State = LOW;
unsigned int last_time_amber_m = 0;

// Global variables and definitions for IMU
float acceleration_x, acceleration_y, acceleration_z;
float angle_x, angle_y, angle_z, last_angle_z;
float angle_offset;
float angle_turned;
bool gyroscope_angle = false, gyro_calibrated = false;
unsigned int last_time_gyroscope_u;

//<-----------------------------------------------------------------------------------------------------------------FUNCTIONS
void check_timeout() {
  if ((current_time_m - start_time_m) >= 2.4E5) timeout = true;
}

void update_motors() {
  leftMotor->setSpeed(leftSpeed);
  rightMotor->setSpeed(rightSpeed);
  leftMotor->run(leftDirection);
  rightMotor->run(rightDirection);
}

void update_location() {
  // Detection of ramp and whether the robot is in starting location or location of dummies
  // search_area == false means that the robot is in the delivery area
  // search_area == true means that the robot is in the search area
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(acceleration_x, acceleration_y, acceleration_z);
    if (on_ramp == false) {
      if (acceleration_y > 0.2) on_ramp = true;
    } else {
      if (acceleration_y < -0.2) on_ramp = false;
      search_area = !search_area;
    }
  }
}

void gyroscope_reset() {
  // Gyroscope_reset sets the angle_offset for the gyroscope and initiates variables for integration
  // It turns the motors off for accurate configuration
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  delay(2000);
  int readings = 0;
  float angle_total = 0;
  while (readings < 20000) {
    if (IMU.gyroscopeAvailable()) IMU.readGyroscope(angle_x, angle_y, angle_z);
    angle_total += angle_z;
    readings += 1;
    angle_offset = angle_total / readings;
  }
  angle_turned = 0;
  last_time_gyroscope_u = micros();
  last_angle_z = angle_z;
}

void measure_gyroscope() {
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(angle_x, angle_y, angle_z);
  }
}

void integrate_gyroscope() {
  unsigned int elapsed_time_u = current_time_u - last_time_gyroscope_u;
  angle_turned += (angle_z + last_angle_z - 2*angle_offset)/2*elapsed_time_u*180/160.7/1E6;
  last_time_gyroscope_u = current_time_u;
  last_angle_z = angle_z;
}

double measure_distance_mm() {
 double* distances = HCSR04.measureDistanceMm();
 return distances[0];
}

//<-------------------------------------------------------------------------------------------------------------SETUP BEGINS
void setup() {
  
  // Configure the motor shield
  while (!AFMS.begin()) { // Check whether the motor shield is properly connected
    digitalWrite(redLED_Pin, HIGH);
  }
  
  // Configure IMU
  while (!IMU.begin()) { // Check whether the IMU works
    digitalWrite(amberLED_Pin, HIGH);
  }
  gyroscope_reset();
  
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
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
  
  // Configure LEDs
  pinMode(amberLED_Pin, OUTPUT);
  pinMode(redLED_Pin, OUTPUT);
  pinMode(greenLED_Pin, OUTPUT);
  digitalWrite(amberLED_Pin, LOW);
  digitalWrite(redLED_Pin, LOW);
  digitalWrite(greenLED_Pin, LOW);
  start_time_m = millis();

}

//<---------------------------------------------------------------------------------------------------------------LOOP BEGINS
void loop() {  
  
  // Time for functions which do not need accurate data
  current_time_m = millis();
  // Time for functions which need accurate data
  current_time_u = micros();

  // Updates all variables
  check_timeout();
  update_location();
  measure_gyroscope();
  if (gyroscope_angle) { // Integrates angle if it should
    integrate_gyroscope();
  }
  update_motors();
  
  // Stops the robot if less than 15cm
  if (!search_area && measure_distance_mm < 150) {
    rightSpeed = 0;
    leftSpeed = 0;
  }

  // Control of the amber light
  if (leftSpeed == 0 && rightSpeed == 0) digitalWrite(amberLED_Pin, LOW);
  else if ((current_time_m - last_time_amber_m) >= 250) {
    digitalWrite(amberLED_Pin, !digitalRead(amberLED_Pin));
    last_time_amber_m = current_time_m;
  }

}
