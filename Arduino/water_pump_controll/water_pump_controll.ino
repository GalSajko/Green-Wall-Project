int DIRECTION_CONTROL_PINS[2] = {2, 4};
int PWM_PINS[3] = {3, 5, 6};

String INIT_MESSAGE = "init";
String INIT_RESPONSE = "OK";
char ON_COMMAND = '1';
char OFF_COMMAND = '0';

char lastCommand;
int lastPumpId;

struct Commands
{
  char command;
  int pumpId;
};

struct Commands parseData(String data)
{
  struct Commands commands;
  if (data[0] == ON_COMMAND || data[0] == OFF_COMMAND)
  {
    commands.command = data[0];
  }
  int pumpId = data[1] - '0';
  if (pumpId < 3 && pumpId >= 0)
  {
    commands.pumpId = pumpId;
  }

  return commands;
}

void pumpControl(char command, int pumpId)
{
  if (command == ON_COMMAND)
  {
    analogWrite(PWM_PINS[pumpId], 255);
  }
  else if (command == OFF_COMMAND)
  {
    analogWrite(PWM_PINS[pumpId], 0);
  }
  
}

void setup() {
  Serial.begin(115200);
  
  digitalWrite(DIRECTION_CONTROL_PINS[0], HIGH);
  digitalWrite(DIRECTION_CONTROL_PINS[1], LOW);
}

void loop() {
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
      struct Commands commands = parseData(data);
      lastCommand = commands.command;
      lastPumpId = commands.pumpId;
    }
  }

  pumpControl(lastCommand, lastPumpId);
}
