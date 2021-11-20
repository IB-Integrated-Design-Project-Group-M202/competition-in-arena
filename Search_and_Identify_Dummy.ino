/* Pragash Mohanarajah (C) November 2021. */

#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select and configure port M1
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
// Select and configure port M2
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

bool aligned = false, gyro_calibrated = false; arrived = false, identified = false;
unsigned long startMillis = 0, currentMillis = 0, currentMicros = 0, lastMicros = 0, elapsedMicros = 0;
const int r_Pin = A0, pt1_Pin = A1, pt2_Pin = A2, greenLED_Pin = 12, redLED_Pin = 13, indicatorDelay = 5000;
float x, y, z, angle_total = 0, angle_offset = 0, angle_turned = 0;
uint2_t dummy = 0;

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
  // Configure green and red indication LEDs as analog outputs
  pinMode(greenLED_Pin, OUTPUT);
  pinMode(redLED_Pin, OUTPUT);
  // Initialise all outputs as LOW
  digitalWrite(greenLED_Pin, LOW);
  digitalWrite(redLED_Pin, LOW);
  
  startMillis = millis();
}

void calibrate_gyro() {
  while (readings < 20000) {
    if (IMU.gyroscopeAvailable()) IMU.readGyroscope(x, y, z);
    angle_total += z; readings += 1; angle_offset = angle_total / readings;
  }
}

void align_with_dummy() {
  // Turn robot right slowly
  leftMotor->setSpeed(51);
  rightMotor->setSpeed(51);
  leftMotor->run(FORWARD);
  rightMotor->run(BACKWARD);
  if (IMU.gyroscopeAvailable()) IMU.readGyroscope(x, y, z);
  currentMicros = micros();
  elapsedMicros = currentMicros - lastMicros;
  angle_turned += (z - angle_offset) * elapsedMicros/1E6 * 180/160.7;
  delay(50);
  lastMicros = currentMicros;
}

void drive_to_dummy() {
  
}

void identify_dummy() {
  
}

void loop() {
  currentMillis = millis();
  time_elapsed = currentMillis - startMillis;
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
