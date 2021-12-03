/*
Naming conventions:
S1 - Sensor 1
m1 - Maximum readout sequence 1
d - Detect mode
s - Stored value
t1 - Time Stamp 1
tm1 - Timer 1
(1 for detection window, 2 for hold)
*/

//sensor interface
const int IRs1 = A0, IRs2 = A1, IRRd=7;
//time parameters
const unsigned long window_time=3000, hold_time=12500;
const int a_size=20;
int s1m1sa[a_size], s2m1sa[a_size];
int a_i=0;
int s1m1sat=0, s1m1saa=0, s2m1sat=0, s2m1saa=0;
//time variables
unsigned long s1m1tm1=0, s1m1tm2=0, s1m1t1=0, s1m1t2=0, s2m1tm1=0, s2m1tm2=0, s2m1t1=0, s2m1t2=0, now=0;

//sensor values
int s1 = 0, s2 =0;
//processed sensor values
int s1m1=0, s1m1s=0, s2m1=0, s2m1s=0;
//loop phase
bool s1m1d = false;

void setup() {
    Serial.begin(2000000);
    for (int i = 0; i < a_size; i++){
        s1m1sa[i]=0;
        s2m1sa[i]=0;
    }
    Serial.println("0\t0\t1023");
}


void loop() {
    s1 = analogRead(IRs1);
    s2 = analogRead(IRs2);
    now = micros();

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
        Serial.print(s1m1saa);
        Serial.print('\t');
        Serial.println(s2m1saa);
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
} 
