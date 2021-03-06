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

void calibrate_pt() {
  now = millis();
  while(millis() - now < 100){
    s1 = analogRead(pt1_Pin); s2 = analogRead(pt2_Pin);
    if(s1 > s1m1){s1m1 = s1;} if(s2 > s1m1){s2m1 = s2;}
    if(s1 < s1m1){s1m2 = s1;} if(s2 < s2m2){s2m2 = s2;}
  }
  pt_calibrated = true;
}

void identify_dummy() {
  if (dummy == 0) {
    if (!pt_calibrated) calibrate_pt();
    N_pt1_maxima = 0; N_pt2_maxima = 0; N_r1_maxima = 0; N_r2_maxima = 0;
    
    now = millis();
    if(s1 >= 500) delay(4); //leave the first spike if there is one
    while(millis() - now < 650){
      s1 = analogRead(pt1_Pin);
      s1 = constrain(s1, s1m2, s1m1);
      s1 = map(s1, s1m2, s1m1, 0, 1023);
      if(s1 >= 200){
        N_pt1_maxima++;
        delayMicroseconds(300);
        if(digitalRead(irr_Pin) == LOW){
          N_r1_maxima++;
        }
        delay(4);
      }
    }

    now = millis();
    if(s2 >= 500){delay(4);} //leave the first spike if there is one
    while(millis() - now < 650){
      s2 = analogRead(pt2_Pin);
      s2 = constrain(s2, s2m2, s2m1);
      s2 = map(s2, s2m2, s2m1, 0, 1023);
      if (s2 >= 200) {
        N_pt2_maxima++;
        delayMicroseconds(300);
        if (digitalRead(irr_Pin) == LOW) {
          N_r2_maxima++;
        }
        delay(4);
      } 
    }

    short percentage_1 = round(float(N_r1_maxima) / float(N_pt1_maxima) * 2);
    short percentage_2 = round(float(N_r2_maxima) / float(N_pt2_maxima) * 2);
    if (last_dummy > 0) dummy = 0;
    else {
      if (percentage_1 == 2 and percentage_2 == 2) dummy = 1; else
      if (percentage_1 == 0 and percentage_2 == 0) dummy = 2; else
      if (percentage_1 == 1 and percentage_2 == 1) dummy = 3;
    }
    last_dummy = dummy;
  }
  if (dummy != 0) {
    if (stopped && !identifiedLine)
      { identifiedLine = true; on_line = false; dummy_angle_1 = dummy_angle; identified_dummy_count ++; dummy_indicator(); }
    if (in_range && !identifiedArea) {
      identifiedArea = true;
      if (dummy_angle_2 != 0 and dummy_angle_3 == 0)
        { dummy_angle_3 = dummy_angle; identified_dummy_count ++; dummy_indicator(); } else
      if (dummy_angle_1 != 0 and dummy_angle_2 == 0)
        { dummy_angle_2 = dummy_angle; identified_dummy_count ++; dummy_indicator(); }
    }
    gyro_calibrated = false;
  }
}

void update_motors() {
  leftMotor->setSpeed(leftSpeed);
  rightMotor->setSpeed(rightSpeed);
  leftMotor->run(leftDirection);
  rightMotor->run(rightDirection);
}

void update_motors_by_direction() {
  if (leftSpeedv>0) leftDirection=FORWARD;
  else leftDirection=BACKWARD;
  if (rightSpeedv>0) rightDirection=FORWARD;
  else rightDirection=BACKWARD;
  update_motors();
}

void update_linesensors(){
  lsc = digitalRead(centralLineSensor);
  lsl = analogRead(leftLineSensor);
  lsr = analogRead(rightLineSensor);
  lsl_mapped = map(min(max(lsl, lsl_min), lsl_max), lsl_min, lsl_max, 0, 255);
  lsr_mapped = map(min(max(lsr, lsr_min), lsr_max), lsr_min, lsr_max, 0, 255);
}

void update_location() {
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
}

void reset_PID(){
  P=0;
  I=0;
  D=0;
  last_P=0;
}

void IR_readout(){
  s1 = analogRead(pt1_Pin);
  s2 = analogRead(pt2_Pin);
  now = micros();
}

void IR_peak_update(){
  if(s1 > s1m1){
    s1m1 = s1;
    s2m1 = s2;
    s1m1t1 = now;
    if(s1m1>s1m1s){
      s1m1s=s1m1;
      s2m1s=s2m1;
      s1m1t2=now;
    }
  }
}

void if_in_range(){
  if(ssum>=in_range_threshold){
    if(in_range==false){
      in_range=true;
      reset_PID();
      centreSpeed=approachSpeed;
    }
  }
  else{
    in_range=false;
    centreSpeed=0;
  }
}

void if_on_line(){
  update_linesensors();
  if(lsc==1){
    if(on_line==false){
      on_line=true;
      reset_PID();
    }
  }
  else{
    on_line=false;
  }
}

void if_cross_road(){
  update_linesensors();
  if(on_line==true && lsr_mapped>=lsr_threshold && lsr_mapped>=lsr_threshold){
    cross_road=true;
  }
  else{
    cross_road=false;
  }
}

void if_arrived(){
  if(ranging_index%16==0){
    distance=measure_distance_mm();
    if(distance>0 and distance<=180){
      arrived=true;
    }
  }
  ranging_index++;
}

void PID_update(){
  gap=now-lastPID;
  gapf=gap/5000;
  if(in_range==true){
    P=sdiff;
    I=I+P*gapf/10;
    D=(P-last_P)/gapf;
    last_P=P;
    speed_difference=max(min((Kp*P+Ki*I+Kd*D), turnSpeed), -turnSpeed);
    if_arrived();
  }
  else{
    speed_difference=turnSpeed;
  }
  lastPID=now;
  leftSpeedv=centreSpeed-speed_difference;
  rightSpeedv=centreSpeed+speed_difference;
  leftSpeed=abs(leftSpeedv);
  rightSpeed=abs(rightSpeedv);
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
  if (angle_change > angle_z_threshold || angle_change < - angle_z_threshold) angle_turned += angle_change*elapsed_time_u*180/161/1E6;
  last_time_gyroscope_u = micros(); last_angle_z = angle_z;
}

void drive_on_line() {
  leftDirection = FORWARD; rightDirection = FORWARD; stopped = false;
  distance = measure_distance_mm();
  update_linesensors();
  
  P=lsl_mapped-lsr_mapped;
  I=I+P;
  D=P-last_P;
  last_P=P;
  
  speed_difference=max(min((Kp*P+Ki*I+Kd*D), 45), -45);
  leftSpeed=centreSpeed - speed_difference;
  rightSpeed=centreSpeed + speed_difference;
  if_on_line();
  update_location();
  update_motors();
  amberLED_control();
}

void drive_on_line_to_obstruction() {
  drive_on_line();
  distance = measure_distance_mm();
  if (distance > 0 && distance < 100) { leftSpeed = 0; rightSpeed = 0; stopped = true; }
  update_motors();
}

void search(){
  search_timer=millis();
  sequence_timeout=false;
  while(!arrived and !sequence_timeout){
    IR_readout();
    IR_peak_update();

    s1m1tm1=now-s1m1t1;

    //timeout
    if(s1m1tm1>=window_time){
      //initialises hold
      s1m1d=false;
      s1m1=0;
      s2m1=0;
      s1m1sat=s1m1sat-s1m1sa[a_i];
      s2m1sat=s2m1sat-s2m1sa[a_i];
      s1m1sat+=s1m1s;
      s2m1sat+=s2m1s;
      s1m1sa[a_i]=s1m1s;
      s2m1sa[a_i]=s2m1s;
      a_i+=1;
      if (a_i>=a_size){
        a_i=0;
      }
      s1m1saa=s1m1sat/a_size;
      s2m1saa=s2m1sat/a_size;
      ssum=s1m1saa+s2m1saa;
      sdiff=s1m1saa-s2m1saa;
      if_in_range();
      PID_update();
      update_motors_by_direction();
      if((now_ms-search_timer)>=search_timeout and !in_range){
        sequence_timeout=true;
      }
    }

    s1m1tm2=now-s1m1t2;
    //timeout
    if(s1m1tm2>=hold_time){
      //initiates detection window
      s1m1d=true;
      s1m1t2=now;
      s1m1s=s1m1;
      s2m1s=s2m1;
    }
    now_ms=millis();
  }
}

void walk(){
  search_timer=millis();
  sequence_timeout=false;
  while(!arrived and !sequence_timeout){
    if_arrived();
    leftSpeed=245;
    rightSpeed=220;
    update_motors();
    now_ms=millis();
    if((now_ms-search_timer)>=3000){
      sequence_timeout=true;
    }
  }
}

void search_and_align_and_identify(){
  search();
  walk();
  if_arrived();
  if(arrived){
    leftSpeed=0;
    rightSpeed=0;
    update_motors();
  }
  if_in_range();
  if(in_range){
    identify_dummy();
  }
}

void escape(){
  arrived=false; in_range = false; identifiedArea = false;
  leftSpeedv=-250;
  rightSpeedv=150;
  leftSpeed=abs(leftSpeedv);
  rightSpeed=abs(rightSpeedv);
  update_motors_by_direction();
  delay(2000);
}

void align_with_line() {
  leftSpeed = 80; rightSpeed = 80;
  short target_angle = dummy_angle_1 + dummy_angle_2 + dummy_angle_3;
  if (angle_turned < target_angle) { leftDirection = BACKWARD; rightDirection = FORWARD; } else
  if (angle_turned > target_angle) { leftDirection = FORWARD; rightDirection = BACKWARD; } else
  if (angle_turned <= target_angle + 0.5 and angle_turned >= target_angle - 0.5)
  { leftSpeed = 0; rightSpeed = 0; aligned = true; gyro_calibrated = false; on_line = false; }
  update_motors();
  amberLED_control();
}

void align_to_line() {
  arrived = false; on_line = false;
  leftSpeed = 80; rightSpeed = 80;
  if (angle_turned < 180) { leftDirection = BACKWARD; rightDirection = FORWARD; } else
  if (angle_turned > 180) { leftDirection = FORWARD; rightDirection = BACKWARD; } else
  if (angle_turned <= 180.5 and angle_turned >= 179.5)
  { leftSpeed = 0; rightSpeed = 0; aligned = true; gyro_calibrated = false; on_line = false; }
  update_motors();
  amberLED_control();
}

void drive_to_line() {
  leftSensorStatus = LineSensorStatus(leftLineSensor);
  centralSensorStatus = LineSensorStatus(centralLineSensor);
  rightSensorStatus = LineSensorStatus(rightLineSensor);
  update_location();
  integrate_gyroscope();
  int SensorStati = leftSensorStatus + centralSensorStatus + rightSensorStatus;
  if (SensorStati > 6) { leftSpeed = 0; rightSpeed = 0; arrived = true; aligned = false; gyro_calibrated = false; on_line = true; }
  if (leftSpeed != 240 && angle_turned <= -2) leftSpeed -= 15;
  if (rightSpeed != 240 && angle_turned >= 2) rightSpeed -= 15;
  if (angle_turned >= -2 && angle_turned <= 2) { leftSpeed = 255; rightSpeed = 255; }
  update_motors();
  amberLED_control();
}
