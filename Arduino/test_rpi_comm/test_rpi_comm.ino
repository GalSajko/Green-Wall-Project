#include <Servo.h>

int NUMBER_OF_LEGS = 5;

int GRIPPERS_CONTROL_PINS[5] = {8, 9, 10, 11, 12};
int GRIPPERS_FEEDBACK_PINS[5] = {A0, A1, A2, A3, A4};
int SWITCH_PINS[5] = {2, 3, 4, 5, 6};

// Commands for controlling a gripper.
char OPEN_COMMAND = 'o';
char CLOSE_COMMAND = 'c';
String INIT_MESSAGE = "init";

// Values when servo is in closed or open position.
int CLOSE_THRESHOLD = 260;
int OPEN_THRESHOLD = 400;

float OPEN_STROKE_MM = 10;
float CLOSED_STROKE_MM = 20;
float MAX_STROKE_MM = 30;

int switchValue;
int servoValue;
Servo grippers[5];

struct CommandGrippersIds {
  char command;
  int gripperId;
};

struct CommandGrippersIds parseData(String data){
  struct CommandGrippersIds cmdGrip;
  if (data[0] == OPEN_COMMAND || data[0] == CLOSE_COMMAND){
    cmdGrip.command = data[0];
  }
  cmdGrip.gripperId = data[1] - '0';
  
  return cmdGrip;
}

String getGrippersOnPositionsString(int gripperId, char command){
  String str;
  int gripperValue = analogRead(GRIPPERS_FEEDBACK_PINS[gripperId]);
  if (gripperValue > OPEN_THRESHOLD && command == OPEN_COMMAND){
    str = "1";
  }
  else if (gripperValue < CLOSE_THRESHOLD && command == CLOSE_COMMAND){
    str = "1";
  }
  else {
    str = "0";
  }
  return str;
}

void setStrokeMm(int gripperId, float strokeDesired){
  int usec = 1000 + strokeDesired * (2000 - 1000) / MAX_STROKE_MM;
  Servo gripper = grippers[gripperId];
  gripper.writeMicroseconds(usec);
}



void setup() {
  Serial.begin(9600);

  for (int i = 0; i < NUMBER_OF_LEGS; i++){
    grippers[i].attach(GRIPPERS_CONTROL_PINS[i]);
    pinMode(SWITCH_PINS[i], INPUT_PULLUP);
  }

  switchValue = 0;
}

void loop() {

  switchValue = digitalRead(SWITCH_PINS[0]);
  Serial.println(switchValue);



  // if (Serial.available() > 0){
  //   String data = Serial.readStringUntil('\n');
    
  //   if (data == INIT_MESSAGE){
  //     Serial.println("1");
  //   }
  //   else {
  //     struct CommandGrippersIds cmdGrip = parseData(data);

  //     if (cmdGrip.command == OPEN_COMMAND){
  //       setStrokeMm(cmdGrip.gripperId, OPEN_STROKE_MM);
  //       delay(2000);
  //     }
  //     else if (cmdGrip.command == CLOSE_COMMAND){
  //       setStrokeMm(cmdGrip.gripperId, CLOSED_STROKE_MM);
  //       delay(2000);
  //     }
  //     String grippersFeedback = getGrippersOnPositionsString(cmdGrip.gripperId, cmdGrip.command);
  //     Serial.println(grippersFeedback);
  //   }
  // }
}
