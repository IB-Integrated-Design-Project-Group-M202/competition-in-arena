#define R_PIN A0
#define PT1_PIN A1
#define PT2_PIN A2

void setup() {
  // Configure two phototransistors as analog inputs
  pinMode(PT1_PIN, INPUT);
  pinMode(PT2_PIN, INPUT);
  // Configure IR receiver as an analog input_pullup, i.e. with inverted logic with respect to 5V V_CC.
  pinMode(R_PIN, INPUT_PULLUP);
}

void loop() {
  
}
