#include <Servo.h>

Servo myServo;

//Constanst for pin connections etc.
int servo1Pin = 8;
float maxStroke = 30;
float minStroke = 0;
float openStroke = 7;
float closedStroke = 20;
float desiredLength = 0;

float LimitStroke(float strokeDesired){
  if (strokeDesired > maxStroke){
    strokeDesired = maxStroke;
  }
  if (strokeDesired < minStroke){
    strokeDesired = minStroke;
  }
  return strokeDesired;
}

void SetStrokeMm(float strokeDesired){
  strokeDesired = LimitStroke(strokeDesired);
  int usec = 1000 + strokeDesired * (2000 - 1000) / maxStroke;
  myServo.writeMicroseconds(usec);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  myServo.writeMicroseconds(1500);
  myServo.attach(servo1Pin);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0){
    String data = Serial.readStringUntil('\n');
    desiredLength = data.toFloat();
    SetStrokeMm(desiredLength);
    if (data == "o"){
      SetStrokeMm(10);
      delay(2000);
      Serial.println("Gripper opened.");
    }
    else if (data == "c"){
      SetStrokeMm(20);
      delay(2000);
      Serial.println("Gripper closed.");
    }
    
  }
}
