#include <Servo.h>

const int NUMBER_OF_LEGS = 5;

int GRIPPERS_CONTROL_PINS[NUMBER_OF_LEGS] = {3, 5, 6, 9, 10};
int GRIPPERS_FEEDBACK_PINS[NUMBER_OF_LEGS] = {A1, A3, A5, A6, A7};
int SWITCH_PINS[5] = {A2, A4, 4, 7, 11};

// Commands for controlling a gripper.
char OPEN_COMMAND = 'o';
char CLOSE_COMMAND = 'c';
String INIT_MESSAGE = "init";

String INIT_RESPONSE = "OK";
char GRIPPER_OPENED_RESPONSE = '1';
char GRIPPER_CLOSED_RESPONSE = '0';
char GRIPPER_MOVING_RESPONSE = '2';

// Values when servo is in closed or open position.
int GRIPPERS_OPEN_THRESHOLD[NUMBER_OF_LEGS] = {670, 670, 670, 670, 670};
int GRIPPERS_CLOSE_THRESHOLD[NUMBER_OF_LEGS] = {490, 490, 490, 490, 490};

float OPEN_STROKE_MM = 0;
float CLOSED_STROKE_MM = 14;
float MAX_STROKE_MM = 30;

int switchValue;
int servoValue;
Servo grippers[NUMBER_OF_LEGS];

struct CommandGrippersIds 
{
  char command;
  int gripperId;
};

struct CommandGrippersIds parseData(String data)
{
  struct CommandGrippersIds cmdGrip;
  if (data[0] == OPEN_COMMAND || data[0] == CLOSE_COMMAND)
  {
    cmdGrip.command = data[0];
  }
  cmdGrip.gripperId = data[1] - '0';
  
  return cmdGrip;
}

void setStrokeMm(int gripperId, float strokeDesired)
{
  // 0 mm -> 1000 usec
  // 30 mm -> 2000 usec
  int k = 1000 / MAX_STROKE_MM;
  int n = 1000;
  int usec = k * strokeDesired + n;
  Servo gripper = grippers[gripperId];
  gripper.writeMicroseconds(usec);
}

// Create string message from grippers states:
// 0 - closed, 1 - open, 2 - in between.
// Example: "11011" - all grippers except third are opened.
String getGrippersStatesMessage(int currentStates[])
{
  char message[NUMBER_OF_LEGS];
  for (int i = 0; i < NUMBER_OF_LEGS; i++)
  {
    if (currentStates[i] < GRIPPERS_CLOSE_THRESHOLD[i])
    {
      message[i] = GRIPPER_CLOSED_RESPONSE;
    }
    else if (currentStates[i] > GRIPPERS_OPEN_THRESHOLD[i])
    {
      message[i] = GRIPPER_OPENED_RESPONSE;
    }
    else if (currentStates[i] < GRIPPERS_OPEN_THRESHOLD[i] && currentStates[i] > GRIPPERS_CLOSE_THRESHOLD[i])
    {
      message[i] = GRIPPER_MOVING_RESPONSE;
    }
  }
  return message;
}
String getSwitchesStatesMessage(int currentStates[])
{
  String message;
  for (int i = 0; i < NUMBER_OF_LEGS; i++)
  {
    message += String(currentStates[i]);
  }
  return message;
}

void setup() 
{
  Serial.begin(115200);
  for (int i = 0; i < NUMBER_OF_LEGS; i++)
  {
    grippers[i].attach(GRIPPERS_CONTROL_PINS[i]);
    pinMode(SWITCH_PINS[i], INPUT_PULLUP);
  }
}

void loop() 
{
  if (Serial.available() > 0)
  {
    String data = Serial.readStringUntil('\n');  

    if (data == INIT_MESSAGE)
    {
      Serial.println(INIT_RESPONSE + '\n');
      return;
    }
    else 
    {
      struct CommandGrippersIds cmdGrip = parseData(data);

      if (cmdGrip.command == OPEN_COMMAND)
      {
        setStrokeMm(cmdGrip.gripperId, OPEN_STROKE_MM);
      }
      else if (cmdGrip.command == CLOSE_COMMAND)
      {
        setStrokeMm(cmdGrip.gripperId, CLOSED_STROKE_MM);
      }
    }
  }

  int grippersStates[NUMBER_OF_LEGS];
  int switchesStates[NUMBER_OF_LEGS];
  for (int i = 0; i < NUMBER_OF_LEGS; i++)
  {
    grippersStates[i] = analogRead(GRIPPERS_FEEDBACK_PINS[i]);
    switchesStates[i] = digitalRead(SWITCH_PINS[i]);
  }

  String grippersMessage = getGrippersStatesMessage(grippersStates);
  String switchesMessage = getSwitchesStatesMessage(switchesStates);
  Serial.println(grippersMessage + switchesMessage + '\n');
  delay(100);
}
