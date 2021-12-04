/* Function is used for identification of the dummies. It detects spikes of voltage from phototransistors and checks
 * whether there is a change of value for IR Receiver. One of the dummies has no such changes, second one has such
 * changes for half of the spikes and the third one has them for all of the spikes. Function looks at about 100 spikes 
 * and determines which dummy it is.
 */

// Define pins for 2 phototransistors and a receiver
#define PT1 A2
#define PT2 A3
#define IR 7

// Variables for sensor values and for detection of dummies
int s1, s2;
int dummy = 0, last_dummy = 0;

// Variables for calibration m1 means maximum, m2 means minimum
unsigned long mil;
int s1m1 = 0, s2m1 = 0, s1m2 = 1023, s2m2 = 1023;

// Variables for identification. Readings are the amount of spikes, true_readings are the amount of spikes that have a change of value from IR sensor
// 1 and 2 indicates which sensor it is
int readings_1 = 0;
int readings_2 = 0;
int true_readings_1 = 0;
int true_readings_2 = 0;

void setup() {
  // Setups the pins as inputs and initiates serial communication
  pinMode(PT1, INPUT);
  pinMode(PT2, INPUT);
  pinMode(IR, INPUT);
  Serial.begin(9600);
  
  // Finds maximum and minimum values of the signals over 100ms 
  // It is then used to strech the inputs to take values from 0 to 1023 instead of some arbitrary values
  mil = millis();
  if(s1 >= 500){
    // Leave the first spike if there is one
    delay(4);
  } 
  while(millis() - mil < 100){
    s1 = analogRead(PT1);
    s2 = analogRead(PT2);
    if(s1 > s1m1){s1m1 = s1;}
    if(s2 > s2m1){s2m1 = s2;}
    if(s1 < s1m2){s1m2 = s1;}
    if(s2 < s2m2){s2m2 = s2;}
  }
}

void loop() {
  
  /*  Looks at the point where signal from phototransistor is higher
   *  than 500 (beginning of a spike)and then checks whether IR sensor gets low for this
   *  spike.
  */
  
  // Initialises the values of readings and true_readings 
  int readings_1 = 0;
  int readings_2 = 0;
  int true_readings_1 = 0;
  int true_readings_2 = 0;
    
  mil = millis();
  s1 = analogRead(PT1);
  if(s1 >= 500){
     //leave the first spike if there is one
    delay(4);
  }
  
  // Function looks for  spikes for 650ms
  while(millis() - mil < 650){
    // Streches the values to range between 0 and 1023
    s1 = analogRead(PT1);
    s1 = constrain(s1, s1m2, s1m1);
    s1 = map(s1, s1m2, s1m1, 0, 1023);
    
    // Looks for a spike and adds 1 to readings_1
    if(s1 >= 200){
      readings_1++;
      delayMicroseconds(300);
      
      // Looks for a change in the value from IR receiver and adds 1 to true_readings_1 if it is LOW
      if(digitalRead(IR) == LOW){
        true_readings_1++;
      }
      // Jump to just before the next spike
      delay(4);
    }
  }
   
  // The same function as earlier, but for the second phototransistor
  mil = millis();
  if(s2 >= 500){
    delay(4);
  }
  while(millis() - mil < 650){
    s2 = analogRead(PT2);
    s2 = constrain(s2, s2m2, s2m1);
    s2 = map(s2, s2m2, s2m1, 0, 1023);
    if (s2 >= 200) {
      readings_2++;
      delayMicroseconds(300);
      if (digitalRead(IR) == LOW) {
        true_readings_2++;
      }
      delay(4);
    } 
  }
  
  // Readings and true_reading interpretation
  short percentage_1 = round(float(true_readings_1) / float(readings_1) * 2);
  short percentage_2 = round(float(true_readings_2) / float(readings_2) * 2);
  if (last_dummy > 0) dummy = 0;
  else {
    if (percentage_1 == 2 and percentage_2 == 2) dummy = 1; else
    if (percentage_1 == 0 and percentage_2 == 0) dummy = 2; else
    if (percentage_1 == 1 and percentage_2 == 1) dummy = 3;
  }
  last_dummy = dummy;
  
  // Prints on serial port for testing
  Serial.print(readings_1); Serial.print('\t');
  Serial.print(readings_2); Serial.print('\t');
  Serial.print(true_readings_1); Serial.print('\t');
  Serial.print(true_readings_2); Serial.print('\t');
  Serial.print(percentage_1); Serial.print('\t');
  Serial.print(percentage_2); Serial.print('\t');
  Serial.println(dummy);
}
