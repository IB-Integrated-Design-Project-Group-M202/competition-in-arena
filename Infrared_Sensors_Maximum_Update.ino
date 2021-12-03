unsigned long s1m1tm1=0, s1m1tm2=0, s1m1t1=0, s1m1t2=0, lastt=0, gap=0, now=0;
int s1 = 0;
int s1m1=0, s1m1s=0;
bool s1m1d = false;
const unsigned long window_time=3000, hold_time=6500;
const int IRs1 = A0;

void setup() {
    Serial.begin(2000000);
}

void loop() {
    s1 = analogRead(IRs1);
    now = micros();

    //hold phase
    if(s1m1d==false){
        s1m1tm2=now-s1m1t2;
        //timeout
        if(s1m1tm2>=hold_time){
            //initiates detection window
            s1m1d=true;
            s1m1t1=now;
        }
    }

    //detection phase
    if(s1m1d==true){
        //maximum value update
        if(s1 > s1m1){
            s1m1 = s1;
            s1m1t1 = now;
        }
        s1m1tm1=now-s1m1t1;

        //timeout
        if(s1m1tm1>=window_time){
            //initialises hold
            s1m1d=false;
            s1m1t2=now;
            s1m1s=s1m1;
            s1m1=0;
        }

    }
  
    gap=now-lastt;
    Serial.print(s1m1tm1);
    Serial.print('\t');
    Serial.print(s1m1tm2);
    Serial.print('\t');
    Serial.print(s1m1d);
    Serial.print('\t');
    Serial.print(s1);
    Serial.print('\t');
    Serial.print(s1m1);
    Serial.print('\t');   
    Serial.print(s1m1s);
    Serial.print('\t'); 
    Serial.println(gap);
    lastt=now;
}
