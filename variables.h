#include <Adafruit_MotorShield.h>
#include <Arduino_LSM6DS3.h>
#include <HCSR04.h>

// Global variables for state of the robot
unsigned long start_time_m, finish_time_m = 3E5;
bool accel = true, decel = false, timeout = false;
bool on_line = true, cross_road = false, on_ramp = false, search_area = false;
bool stopped = false, reached = false, aligned = false, arrived = false, identifiedLine = false, identifiedArea = false;

// Global variables and definitions for motors
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the motor shield object with the default I2C address
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1); // Select and configure port M1
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2); // Select and configure port M2
uint8_t leftSpeed = 0, rightSpeed = 0, leftDirection = FORWARD, rightDirection = FORWARD, trigger_duration = 10;

// Global variables and definitions for distance sensor
#define echoPin 2
#define trigPin 3
double* distances;
unsigned short distance;

// Global variables and definitions for line sensors
#define leftLineSensor A3
#define centralLineSensor A4
#define rightLineSensor A5
short leftSensorStatus, centralSensorStatus, rightSensorStatus;

// Global variables and definitions for IR sensors
#define irr_Pin A0
#define pt1_Pin A1
#define pt2_Pin A2
const int numReadings = 576;
int readings[numReadings];
int readIndex = 0, total = 0, average = 0, pt_Min = 1023, pt_Max = 0;
bool pt1_maximum = false, pt2_maximum = false;

// Global variables and definitions for LEDs and Dummy Indication
#define amberLED_Pin 8
#define greenLED_Pin 12
#define redLED_Pin 13
uint8_t amberLED_duration = 250, indicatorDelay = 1000;
short amberLED_State = LOW, dummy = 0, identified_dummy_count = 0;
unsigned int last_time_amber_m = 0;

// Global variables and definitions for IMU
float acceleration_x, acceleration_y, acceleration_z;
float angle_x, angle_y, angle_z, last_angle_z;
float angle_offset;
float angle_turned, left = 0, right = 0, dummy_angle;
bool gyro_calibrated = false;
unsigned int last_time_gyroscope_u;

struct states { // For monitoring the robot status
  bool gyro_calibrated;
  bool search_area;
  bool stopped;
  bool identifiedLine;
  bool pt1_maximum;
  bool pt2_maximum;
  bool aligned;
  bool arrived;
  bool identifiedArea;
  unsigned short distance;
  short amberLED_State;
  short leftSensorStatus;
  short centralSensorStatus;
  short rightSensorStatus;
  uint8_t leftSpeed;
  uint8_t rightSpeed;
};
