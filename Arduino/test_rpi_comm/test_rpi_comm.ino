#include <Servo.h>

Servo myServo;

// Commands for controlling a gripper.
const String openCommand = "o";
const String closeCommand = "c";

// Values when servo is in closed or open position.
const int closeThreshold = 260;
const int openThreshold = 400;

// Constants for pin connections etc.
const int servo1Pin = 8;
const int servo1FeedbackPin = A0;
const int switchPin = 9;
const float maxStroke = 30;
const float minStroke = 0;
const float openStroke = 10;
const float closedStroke = 20;

float desiredLength;
int switchValue;
int servoValue;

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
  myServo.writeMicroseconds(1000);
  myServo.attach(servo1Pin);

  pinMode(switchPin, INPUT_PULLUP);
}

void loop() {
  
//  switchValue = digitalRead(switchPin);
//  Serial.println(switchValue);

  if (Serial.available() > 0){
    String data = Serial.readStringUntil('\n');

    if (data == openCommand){
      setStrokeMm(openStroke);
      delay(2000);
      servoValue = analogRead(servo1FeedbackPin);
      if (servoValue > openThreshold){
        Serial.println("1");
      }
      else{
        Serial.println("0");
      }
    }
    else if (data == closeCommand){
      setStrokeMm(closedStroke);
      delay(2000);
      servoValue = analogRead(servo1FeedbackPin);
      if (servoValue < closeThreshold){
        Serial.println("1");
      }
      else{
        Serial.println("0");
      }
    }
    
  }
}
