#include <Servo.h>

Servo myServo;

// Commands for controlling a gripper.
const String openCommand = "o";
const String closeCommand = "c";

// Constants for pin connections etc.
const int servo1Pin = 8;
const int switchPin = 9;
const float maxStroke = 30;
const float minStroke = 0;
const float openStroke = 10;
const float closedStroke = 20;

float desiredLength;
int switchValue;

float limitStroke(float strokeDesired){
  if (strokeDesired > maxStroke){
    strokeDesired = maxStroke;
  }
  else if (strokeDesired < minStroke){
    strokeDesired = minStroke;
  }
  return strokeDesired;
}

void setStrokeMm(float strokeDesired){
  strokeDesired = limitStroke(strokeDesired);
  int usec = 1000 + strokeDesired * (2000 - 1000) / maxStroke;
  myServo.writeMicroseconds(usec);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  myServo.writeMicroseconds(1500);
  myServo.attach(servo1Pin);

  pinMode(switchPin, INPUT);
}

void loop() {
  switchValue = digitalRead()
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0){
    String data = Serial.readStringUntil('\n');
    desiredLength = data.toFloat();

    setStrokeMm(desiredLength);

    if (data == openCommand){
      setStrokeMm(openStroke);
      delay(2000);
      Serial.println("Gripper opened.");
    }
    else if (data == closeCommand){
      setStrokeMm(closedStroke);
      delay(2000);
      Serial.println("Gripper closed.");
    }
    
  }
}
