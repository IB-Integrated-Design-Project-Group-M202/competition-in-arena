const float tangent_slope_angle = 0.2;
bool on_ramp = false, location_changed = false, search_area = false;
float acceleration_x, acceleration_y, acceleration_z;

void setup() {
  // Initiate Serial Communication for monitoring
  Serial.begin(9600);
  
  // Configure IMU
  if (!IMU.begin()) {
    // Check whether the IMU works
    while(1);
  }
}

void loop() {
  /* Detection of ramp and whether the robot is in starting location or location of dummies
   * search_area == false means that the robot is in the delivery area
   * search_area == true means that the robot is in the search area
   */
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(acceleration_x, acceleration_y, acceleration_z);
    if (!on_ramp) {
      if (acceleration_y > tangent_slope_angle) { on_ramp = true; location_changed = true; } else
      if (acceleration_y < - tangent_slope_angle)
        {  on_ramp = true; if (location_changed) { search_area = !search_area; location_changed = false; } }
    } else {
      if (acceleration_y < tangent_slope_angle && acceleration_y > - tangent_slope_angle) on_ramp = false;
    }
  }
  delay(50); // Add delay for stability. In execution.ino, other function calls will introduce delays.
  
  // Print Location Updates after each loop.
  Serial.print(acceleration_x);
  Serial.print('\t');
  Serial.print(acceleration_y);
  Serial.print('\t');
  Serial.print(acceletation_z);
  Serial.print('\t');
  Serial.print(on_ramp);
  Serial.print('\t');
  Serial.print(location_changed);
  Serial.print('\t');
  Serial.println(search_area);
}
