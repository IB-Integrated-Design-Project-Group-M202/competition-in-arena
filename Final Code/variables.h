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
uint8_t leftSpeed = 0, centreSpeed = 200, rightSpeed = 0, leftDirection = FORWARD, rightDirection = FORWARD;

// Global variables and definitions for distance sensor
#define echoPin 2
#define trigPin 3
double* distances;
unsigned short distance;

// Global variables and definitions for line sensors
#define leftLineSensor A2
#define centralLineSensor 4
#define rightLineSensor A3
const short lsl_max = 380, lsr_max = 920;
const short lsl_min = 64, lsr_min = 122;
const uint8_t lsl_threshold = 150;
const uint8_t lsr_threshold = 150;
short lsl = 0, lsc = 0, lsr = 0;
uint8_t lsl_mapped = 0, lsr_mapped = 0;
short leftSensorStatus, centralSensorStatus, rightSensorStatus;

// Global variables and definitions for IR sensors
#define irr_Pin 7
#define pt1_Pin A0
#define pt2_Pin A1
const unsigned long window_time = 3000, hold_time = 12500;
const int a_size = 20, in_range_threshold=750;
const float Kp = 0.04, Ki = 0.001, Kd = 0.1;
const uint8_t approachSpeed=170, turnSpeed=170;
const unsigned long search_timeout=10000;
int s1m1sa[a_size], s2m1sa[a_size];
int a_i = 0, i = 0, j = 0, s1 = 0, s2 = 0, s1m1 = 0, s1m2 = 1023, s1m1s = 0, s2m1 = 0, s2m2 = 1023;
int s2m1s = 0, s1m1sat = 0, s1m1saa = 0, s2m1sat = 0, s2m1saa = 0, sdiff = 0, ssum = 0;
short leftSpeedv = 0, rightSpeedv = 0, speed_difference = 0;
uint8_t ranging_index=0;
int N_pt1_maxima = 0, N_pt2_maxima = 0, N_r1_maxima = 0, N_r2_maxima = 0, pt1_maxima[3], pt2_maxima[3];
float gapf = 0;
long P=0, I=0, D=0, last_P=0;
unsigned long s1m1tm1 = 0, s1m1tm2 = 0, s1m1t1 = 0, s1m1t2 = 0, s2m1tm1 = 0, s2m1tm2 = 0, s2m1t1 = 0, s2m1t2 = 0, lastPID = 0, gap = 0, now = 0;
unsigned long search_timer=0, now_ms=0;
bool s1m1d = false, in_range = false, sequence_timeout=false;;
bool pt_calibrated = false, pt1_maximum = false, pt2_maximum = false;

// Global variables and definitions for LEDs and Dummy Indication
#define amberLED_Pin 8
#define greenLED_Pin 12
#define redLED_Pin 13
const uint8_t amberLED_duration = 250;
const short indicatorDelay = 3000;
short amberLED_State = LOW, dummy = 0, last_dummy = 0, identified_dummy_count = 0;
unsigned int last_time_amber_m = 0;

// Global variables and definitions for IMU
const float tangent_slope_angle = 0.15, angle_z_threshold = 0.15;
float acceleration_x, acceleration_y, acceleration_z;
float angle_x, angle_y, angle_z, last_angle_z;
float angle_offset;
float angle_turned, left = 0, right = 0, pt1_angle, pt2_angle, dummy_angle, dummy_angle_1, dummy_angle_2, dummy_angle_3;
bool gyro_calibrated = false;
unsigned int last_time_gyroscope_u;

struct states { // For monitoring the robot status
  bool gyro_calibrated;
  bool search_area;
  bool stopped;
  bool identifiedLine;
  bool pt1_maximum;
  bool pt2_maximum;
  bool in_range;
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
