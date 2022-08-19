int DIRECTION_CONTROL_PINS[2] = {2, 4};
int PWM_PINS[3] = {3, 5, 6};


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  digitalWrite(DIRECTION_CONTROL_PINS[0], HIGH);
  digitalWrite(DIRECTION_CONTROL_PINS[1], LOW);
}

void loop() {
  // put your main code here, to run repeatedly:

  analogWrite(PWM_PINS[0], 255);
}
