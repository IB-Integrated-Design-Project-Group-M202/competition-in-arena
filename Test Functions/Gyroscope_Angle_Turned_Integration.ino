/* Tomasz Twardoch Nov 2021
 * 
 * This function uses yaw supplied by the gyroscope on board of Arduino Uno Rev 2 to find the change of angle
 * The function needs a calibration first, because otherwise the offset would make the answer unstable after the integration of yaw
 */

#include <Arduino_LSM6DS3.h>

/* Initialise variables, readings and angle_total are used for initial calibration of the microcontroller.
 * x, y are only used in data fetching function that needs 3 variables (we only use readings for z axis named angle)
 * angle_offset is set during calibration and is then used to increase the accuracy of the gyroscope
 * angle_turned is the output stating the change of angle of the Arduino in the z axis since the calibration
 */
int readings = 0;
float x, y, angle, angle_offset = 0, angle_turned = 0, angle_total = 0;
// Values below 0.2 are ommited as the values of angle during use are much bigger. Values below 0.2 are mostly errors while the Arduino does not move 
const float angle_threshold = 0.2;

void setup() {
  // Serial used for testing
  Serial.begin(9600);
  while (!Serial);
  
  // Initialising IMU with the gyroscope
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }
  
  // Printing some initial data to the serial com
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");

  
  // Calibration loop. It takes 20,000 measurements in a few seconds to find the offset
  while (readings < 20000) {
    if (IMU.gyroscopeAvailable()) IMU.readGyroscope(x, y, angle);
    angle_total += angle;
    readings ++;
    angle_offset = angle_total / readings;
  }
}

// Variables for time
unsigned long currentMicros, lastMicros = 0, elapsedMicros;

void loop() {
  // If a reading is available, the function integrates it to find the angle_turned
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, angle);
    
    // Finds out how much time passed since the last measurement
    currentMicros = micros();
    elapsedMicros = currentMicros - lastMicros;
    
    // Integrates the angle taking the angle_offset into consideration
    // There is another offset of 180/160.7 found experimentally
    // It integrates the value only if the value is higher than the threshold
    if(abs(angle - angle_offset) > angle_threshold){
      angle_turned += (angle - angle_offset)*elapsedMicros/1000*180/160.7/1000;
    }
    
    // Delay simulating other functions running in parallel
    delay(50);
    
    // Updates the value of lastMicros for the next loop
    lastMicros = currentMicros;
    
    // Prints values for testing
    Serial.println(angle-angle_offset);
    Serial.print('\t');
    Serial.println(angle_turned);
    Serial.print('\t');
    Serial.println(elapsedMicros);
    
  }
}
