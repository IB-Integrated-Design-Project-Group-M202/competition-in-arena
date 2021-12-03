/*line sensor analogue*/
#include <Adafruit_MotorShield.h>

bool accel = true, decel = false;

uint8_t centreSpeed=0; //average speed of left and right wheel for updatting the motors

//pin definition 
const int lsc_pin=5,  lsl_pin=A2, lsr_pin=A3;


//sensor readouts
int lsc=0, lsl=0, lsr=0;
uint8_t lsl_mapped=0, lsr_mapped=0;

//floor and celling
int lsl_max=380, lsr_max=920;
int lsl_min=64, lsr_min=122;

//thresholds
const uint8_t lsl_threshold = 150;
const uint8_t lsr_threshold = 150;

//mode
bool on_line=false;
bool junction=false;

//PID parameters
const float Kp=0.2;
const float Ki=0.01;
const float Kd=0.01;

//PID variables
int P=0, I=0, D=0, last_P=0;

int speed_difference=0;
uint8_t left_motor_speed, right_motor_speed;



void update_linesensors(){
    lsc = digitalRead(lsc_pin);
    lsl = analogRead(lsl_pin);
    lsr = analogRead(lsr_pin);
    lsl_mapped = map(min(max(lsl, lsl_min), lsl_max), lsl_min, lsl_max, 0, 255);
    lsr_mapped = map(min(max(lsr, lsr_min), lsr_max), lsr_min, lsr_max, 0, 255);
}

/*void callibrate(){
}*/


void reset_PID(){
    P=0;
    I=0;
    D=0;
    last_P=0;
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

void if_junction(){
    update_linesensors();
    if(on_line==true && lsr_mapped>=lsr_threshold && lsr_mapped>=lsr_threshold){
        junction=true;
    }
    else{
        junction=false;
    }
} 

void setup(){
    Serial.begin(9600);
    Serial.println("0\t0\t0\t255");


}



void loop(){

    update_linesensors();

    P=lsl_mapped-lsr_mapped;
    I=I+P;
    D=P-last_P;
    last_P=P;
    
    speed_difference=max(min((Kp*P+Ki*I+Kd*D), 45), -45);

    left_motor_speed=centreSpeed - speed_difference;
    right_motor_speed=centreSpeed + speed_difference;

    Serial.print(lsl);
    Serial.print("\t");
    Serial.print(lsl_mapped);
    Serial.print("\t");
    Serial.print(left_motor_speed);
    Serial.print("\t");
    Serial.print(lsc*1023);
    Serial.print("\t");
    Serial.print(on_line*1023);
    Serial.print("\t");
    Serial.print(lsr);
    Serial.print("\t");
    Serial.print(lsr_mapped);
    Serial.print("\t");
    Serial.println(right_motor_speed);
}
