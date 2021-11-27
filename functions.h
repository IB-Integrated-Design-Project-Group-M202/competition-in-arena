double measure_distance_mm() {
 double* distances = HCSR04.measureDistanceMm();
 return distances[0];
}

short LineSensorStatus(int LineSensorPin) {
  return map(analogRead(LineSensorPin), 0, 1023, 0, 4);
}

void check_timeout() {
  if ((millis() - start_time_m) >= 2.4E5) {
    timeout = true;
  }
}

void update_motors() {
  leftMotor->setSpeed(leftSpeed);
  rightMotor->setSpeed(rightSpeed);
  leftMotor->run(leftDirection);
  rightMotor->run(rightDirection);
}

void update_location() {
  /* Detection of ramp and whether the robot is in starting location or location of dummies
   * search_area == false means that the robot is in the delivery area
   * search_area == true means that the robot is in the search area
   */
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(acceleration_x, acceleration_y, acceleration_z);
    if (!on_ramp) {
      if (acceleration_y > 0.2) on_ramp = true; else
      if (acceleration_y < -0.2) { on_ramp = true; search_area = !search_area; }
    } else {
      if (acceleration_y < 0.2 && acceleration_y > -0.2) on_ramp = false;
    }
  }
}

void amberLED_control() {
  unsigned short current_time_m = millis();
  if (leftSpeed == 0 && rightSpeed == 0) digitalWrite(amberLED_Pin, LOW);
  else if ((current_time_m - last_time_amber_m) >= 250) {
    digitalWrite(amberLED_Pin, !digitalRead(amberLED_Pin));
    last_time_amber_m = current_time_m;
  }
}

void reset_gyroscope() {
  // Sets the angle_offset for the gyroscope and initiates variables for integration
  // It turns the motors off for accurate configuration
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  delay(500);
  unsigned int reading = 0;
  float angle_total = 0;
  while (reading < 65535) {
    if (IMU.gyroscopeAvailable()) IMU.readGyroscope(angle_x, angle_y, angle_z);
    angle_total += angle_z;
    reading += 1;
    angle_offset = angle_total / reading;
  }
  angle_turned = 0;
  last_time_gyroscope_u = micros();
  last_angle_z = angle_z;
  gyro_calibrated = true;
}

void measure_gyroscope() {
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(angle_x, angle_y, angle_z);
  }
}

void integrate_gyroscope() {
  unsigned int elapsed_time_u = micros() - last_time_gyroscope_u;
  float angle_change = (angle_z + last_angle_z)/2 - angle_offset;
  if (!stopped) {
    if (angle_change > 0.15 || angle_change < -0.15) angle_turned += angle_change*elapsed_time_u*180/161/1E6;
  } else {
    if (angle_change > 0.15 || angle_change < -0.15) angle_turned += angle_change*elapsed_time_u*180/54/1E6;
  }
  last_time_gyroscope_u = micros(); last_angle_z = angle_z;
}

void drive_on_line() {
  leftDirection = FORWARD; rightDirection = FORWARD;
  leftSensorStatus = LineSensorStatus(leftLineSensor);
  centralSensorStatus = LineSensorStatus(centralLineSensor);
  rightSensorStatus = LineSensorStatus(rightLineSensor);
  update_location();
  integrate_gyroscope();
  if (centralSensorStatus > 1) {
    if (leftSensorStatus <= 1 && rightSensorStatus <= 1) {
      cross_road = false;
      if (leftSpeed == rightSpeed) {
        if (angle_turned <= 2 && angle_turned >= -2) { leftSpeed = 255; rightSpeed = 255; }
      } else
      if (leftSpeed != rightSpeed) {
        if (angle_turned >= -2 && angle_turned <= 2) on_line = true;
        if (on_line) { leftSpeed = (leftSpeed + rightSpeed)/2; rightSpeed = leftSpeed; }
      }
    } else
    if (leftSensorStatus > 1 && rightSensorStatus <= 1) {
      on_line = false; leftSpeed -= 25; left = angle_turned;
      if (right != 0 && (angle_turned + right) > 0) { leftSpeed -= 25; rightSpeed += 25; }
    } else
    if (leftSensorStatus <= 1 && rightSensorStatus > 1) {
      on_line = false; rightSpeed -= 25; right = angle_turned;
      if (left != 0 && (angle_turned + left) < 0) { leftSpeed += 25; rightSpeed -= 25; }
    } else
    if (leftSensorStatus > 1 && rightSensorStatus > 1) cross_road = true;
  }
  update_motors();
  amberLED_control();
}

void drive_to_dummy() {
  update_location();
  integrate_gyroscope();
  distance = measure_distance_mm();
  if (distance > 0 && distance < 150) { leftSpeed = 0; rightSpeed = 0; arrived = true; }
  if (leftSpeed != 240 && angle_turned <= -2) { on_line = false; leftSpeed -= 15; }
  if (rightSpeed != 240 && angle_turned >= 2) { on_line = false; rightSpeed -= 15; }
  if (angle_turned >= -2 && angle_turned <= 2) { on_line = true; leftSpeed = 255; rightSpeed = 255; }
  update_motors();
  amberLED_control();
}

void drive_on_line_to_dummy() {
  drive_on_line();
  drive_to_dummy();
  if (arrived) { arrived = false; stopped = true; }
}

void pt_maxima() {
  s1 = analogRead(pt1_Pin);
  s2 = analogRead(pt2_Pin);
  now = micros();

  if (s1 > s1m1) {
    s1m1 = s1;
    s2m1 = s2;
    s1m1t1 = now;
    if (s1m1 > s1m1s) {
       s1m1s = s1m1;
       s2m1s = s2m1;
       s1m1t2 = now;
    }
  }
  s1m1tm1 = now - s1m1t1;

  // Window Timeout
  if (s1m1tm1 >= window_time) {
    //initialises hold
    s1m1d = false;
    s1m1 = 0;
    s2m1 = 0;

    s1m1sat = s1m1sat - s1m1sa[a_i];
    s2m1sat = s2m1sat - s2m1sa[a_i];
    s1m1sat += s1m1s;
    s2m1sat += s2m1s;
    s1m1sa[a_i] = s1m1s;
    s2m1sa[a_i] = s2m1s;
    a_i += 1;
    if (a_i >= a_size) {
      a_i = 0;
    }
    s1m1saa = s1m1sat / a_size;
    s2m1saa = s2m1sat / a_size;
  }
  s1m1tm2 = now - s1m1t2;
  
  // Hold Timeout
  if (s1m1tm2 >= hold_time) {
    //initiates detection window
    s1m1d = true;
    s1m1t2 = now;
    s1m1s = s1m1;
    s2m1s = s2m1;
  }

  // Record Angle for PT1 Maximum
  if (s1m1saa > pt1_maxima[i]) {
    pt1_maxima[i] = s1m1saa; i += 1; if (i == 3) i = 0;
  } else {
    pt1_angle = angle_turned; pt1_maximum = true;
  }
  // Record Angle for PT2 Maximum
  if (s2m1saa > pt1_maxima[i]) {
    pt2_maxima[j] = s2m1saa; j += 1; if (j == 3) j = 0;
  } else {
    pt2_angle = angle_turned; pt2_maximum = true;
  }
  // Calculate Dummy Angle
  if (pt1_maximum && pt2_maximum) dummy_angle = (pt1_angle + pt2_angle) / 2;
}

void align_with_dummy() {
  leftSpeed = 80; rightSpeed = 80;
  if (!pt1_maximum || !pt2_maximum) pt_maxima();
  short angle_error = angle_turned - dummy_angle;
  if (dummy_angle == 0) { leftDirection = BACKWARD; rightDirection = FORWARD; }
  else if (dummy_angle != 0 && angle_error < 0) { leftDirection = FORWARD; rightDirection = BACKWARD; }
  else if (dummy_angle != 0 && angle_error > 0) { leftDirection = BACKWARD; rightDirection = FORWARD; }
  if (dummy_angle != 0 && abs(angle_error) <= 0.5) { leftSpeed = 255; rightSpeed = 255; leftDirection = RELEASE; rightDirection = RELEASE; aligned = true; }
  update_motors();
  amberLED_control();
}

void dummy_indicator() {
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

void identify_dummy() {
  dummy_indicator();
}
