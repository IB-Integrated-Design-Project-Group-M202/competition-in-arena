#define PT1 A2
#define PT2 A3
#define IR 7

//sensor values
int s1, s2;
int dummy = 0, last_dummy = 0;
//Calibration
unsigned long mil;
int s1m1 = 0, s2m1 = 0, s1m2 = 1023, s2m2 = 1023;
int readings_1 = 0;
int readings_2 = 0;
int true_readings_1 = 0;
int true_readings_2 = 0;

void setup() {
  pinMode(PT1, INPUT);
  pinMode(PT2, INPUT);
  pinMode(IR, INPUT);
  Serial.begin(9600);
  
  mil = millis();
  if(s1 >= 500){delay(4);} //leave the first spike if there is one
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
  
  /* Look at the point where signal from phototransistor is higher
   *  than 500 (beginning of a spike)and then checks whether IR sensor gets low for this
   *  spike.
  */

  int readings_1 = 0;
  int readings_2 = 0;
  int true_readings_1 = 0;
  int true_readings_2 = 0;
    
    mil = millis();
    if(s1 >= 500) delay(4); //leave the first spike if there is one
    while(millis() - mil < 650){
      s1 = analogRead(PT1);
      s1 = constrain(s1, s1m2, s1m1);
      s1 = map(s1, s1m2, s1m1, 0, 1023);
      if(s1 >= 200){
        readings_1++;
        delayMicroseconds(300);
        if(digitalRead(IR) == LOW){
          true_readings_1++;
        }
        delay(4);
      }
    }

    mil = millis();
    if(s2 >= 500){delay(4);} //leave the first spike if there is one
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

    short percentage_1 = round(float(true_readings_1) / float(readings_1) * 2);
    short percentage_2 = round(float(true_readings_2) / float(readings_2) * 2);
    if (last_dummy > 0) dummy = 0;
    else {
      if (percentage_1 == 2 and percentage_2 == 2) dummy = 1; else
      if (percentage_1 == 0 and percentage_2 == 0) dummy = 2; else
      if (percentage_1 == 1 and percentage_2 == 1) dummy = 3;
    }
    last_dummy = dummy;
    
  Serial.print(readings_1); Serial.print('\t');
  Serial.print(readings_2); Serial.print('\t');
  Serial.print(true_readings_1); Serial.print('\t');
  Serial.print(true_readings_2); Serial.print('\t');
  Serial.print(percentage_1); Serial.print('\t');
  Serial.print(percentage_2); Serial.print('\t');
  Serial.println(dummy);
}
