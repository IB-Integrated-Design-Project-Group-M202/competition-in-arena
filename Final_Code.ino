#include <Adafruit_MotorShield.h>
#include <Arduino_LSM6DS3.h>
#include <HCSR04.h>


//Global variables for state of the robot
bool accel = true, decel = false, over_ramp = false, on_line = true, dummy_reached = false;

//Global variables and definitions for motors
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the motor shield object with the default I2C address
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1); // Select and configure port M1
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2); // Select and configure port M2
uint8_t leftSpeed = 0, rightSpeed = 0, trigger_duration = 10;

//Global variables and definitions for distance sensor
#define echoPin 2
#define trigPin 3
double* distances;

//Global variables and definitions for line sensors
#define leftLineSensor 4
#define rightLineSensor 7
int leftSensorStatus = LOW, rightSensorStatus = LOW;

//Global variables and definitions for IR sensors
#define pt1_Pin A1
#define pt2_Pin A2
#define TSOP //<-------------------------------------------------------------specify PIN and name if needed and other stuff
int numReadings = 576;
int readings[numReadings];
int readIndex = 0, total = 0, average = 0, pt_Min = 1023, pt_Max = 0;

//Global variables and definitions for LEDs
#define amberLED_Pin 8
#define greenLED_Pin //<-------------------------------------------------------------specify PIN
#define redLED_Pin //<-------------------------------------------------------------specify PIN
uint8_t amberLED_duration = 250;
int amberLED_State = LOW;

//Global variables and definitions for IMU
float x, y, z;


void setup() {  //<-----------------------------------------------------------------------------------------------SETUP BEGINS
  
  // Configure the motor shield
  if (!AFMS.begin()) {         // Check whether the motor shield is properly connected
    //<----------------------------------------------------------appropriate signal for disconnected motors, do we need that?
    while (1);
  }
  leftMotor->setSpeed(150);
  rightMotor->setSpeed(150);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);

  // Configure the distance sensor
  HCSR04.begin(trigPin, echoPin);
  digitalWrite(trigPin, LOW);

  //Configure line sensors
  pinMode(leftLineSensor, INPUT);
  pinMode(rightLineSensor, INPUT);
  
  //Configure IR sensors
  pinMode(pt1_Pin, INPUT);
  pinMode(pt2_Pin, INPUT);
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
  
  //Configure LEDs
  pinMode(amberLED_Pin, OUTPUT);
  pinMode(redLED_Pin, OUTPUT);
  pinMode(greenLED_Pin, OUTPUT);
  digitalWrite(amberLED_Pin, LOW);
  digitalWrite(redLED_Pin, LOW);
  digitalWrite(greenLED_Pin, LOW);

  //Configure IMU
  if (!IMU.begin()) { // Check whether the IMU works
    //<----------------------------------------------------------appropriate signal for failure of IMU, do we need that?
    while (1);
  }

}

void loop() {  //<-----------------------------------------------------------------------------------------------LOOP BEGINS
  // put your main code here, to run repeatedly:

}
