#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

/*BNO calibration data.*/
adafruit_bno055_offsets_t CALIB_DATA = 
{
  -1, 19, -21, 0, 0, 0, 2, -1, 2, 1000, 480
};

/*Pins declaration.*/
int DIRECTION_CONTROL_PINS[2] = {2, 4};
int PWM_PINS[3] = {3, 5, 6};
int NUMBER_OF_PUMPS = 3;

/*Commands to communicate with PC.*/
String INIT_MESSAGE = "init";
String INIT_RESPONSE = "OK";
char PUMP_OFF_COMMAND = '0';
char PUMP_ON_COMMAND = '1';
char INIT_BNO = '2';
char READ_BNO_RPY = '3';

/*Analog voltage for controlling the water pumps.*/
int PUMP_ON_VOLTAGE = 250;
int PUMP_OFF_VOLTAGE = 0;



struct Eulers 
{
  float roll = 0.0;
  float pitch = 0.0;
  float yaw = 0.0;
} init_rpy;

struct PcCommands
{
  char command;
  int pumpId;
};

/*Substract starting rpy values from eulers.*/
void substractInitRpyValues(Eulers *eulers)
{
  eulers->pitch -= init_rpy.pitch;
  eulers->roll -= init_rpy.roll;
  eulers->yaw -= init_rpy.yaw;
}

/*Convert quaternion into eulers.*/
Eulers getEulerAnglesFromQuaternion(imu::Quaternion quaternion, bool doSubstract = true)
{
    Eulers eulers;
    float q1 = quaternion.x();
    float q2 = quaternion.y();
    float q3 = quaternion.z();
    float q0 = quaternion.w();

    eulers.pitch = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
    // eulers.yaw = asin(2 * (q0 * q2 - q3 * q1)) * (-1);
    // eulers.yaw = asin(2 * (q0 * q2 - q3 * q1));
    // eulers.roll = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
    eulers.roll = asin(2 * (q0 * q2 - q3 * q1));
    eulers.yaw = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));

    if (doSubstract)
    {
      substractInitRpyValues(&eulers);
    }
  
    return eulers;
}

String addPlusSigns(String a, String b, String c)
{
  if (a[0] != '-')
  {
    a = '+' + a;
  }
  if (b[0] != '-')
  {
    b = '+' + b;
  }
  if (c[0] != '-')
  {
    c = '+' + c;
  }

  String abc = a + b + c;

  return abc;
  }

/*Create message from eulers, to send on PC.*/
String getEulersMessage(Eulers eulers)
{
  String roll = String(eulers.roll);
  String pitch = String(eulers.pitch);
  String yaw = String(eulers.yaw);

  String rpy = addPlusSigns(roll, pitch, yaw);

  return rpy;
}

String getGravityVectorMessage(sensors_event_t event)
{
  // String x = String(event.acceleration.x * (-1));
  String x = String(event.acceleration.x);
  String y = String(event.acceleration.y);
  String z = String(event.acceleration.z);

  String gravity = addPlusSigns(x, y, z);

  return gravity;
}

/*Parse message from PC into commands.*/
struct PcCommands parseData(String data)
{
  struct PcCommands commands;
  if (data[0] == PUMP_ON_COMMAND || data[0] == PUMP_OFF_COMMAND)
  {
    commands.command = data[0];
    int pumpId = data[1] - '0';
    if (pumpId < 3 && pumpId >= 0)
    {
      commands.pumpId = pumpId;
    }
  }
  else if (data[0] == INIT_BNO || data[0] == READ_BNO_RPY)
  {
    commands.command = data[0];
  }

  return commands;
}

/*Controll water pumps.*/
void pumpControl(char command, int pumpId)
{
  int value;
  if (command == PUMP_ON_COMMAND)
  {
    value = PUMP_ON_VOLTAGE;
  }
  else if (command == PUMP_OFF_COMMAND)
  {
    value = PUMP_OFF_VOLTAGE;
  }

  analogWrite(PWM_PINS[pumpId], value);
}

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() 
{
  Serial.begin(115200);

  if (!bno.begin(OPERATION_MODE_CONFIG))
  {
    Serial.print("No BNO055 detected.");
    while(1);
  }

  // Load BNO calibration.
  bno.setSensorOffsets(CALIB_DATA);
  // Put BNO in IMU mode (relative heading).
  bno.setMode(OPERATION_MODE_IMUPLUS);

  delay(1000);

  // Remap BNO axis to match spider's orientation on the wall.
  bno.setAxisRemap(bno.REMAP_CONFIG_P2);
  bno.setAxisSign(bno.REMAP_SIGN_P2);
  // bno.setAxisSign(bno.REMAP_SIGN_P1);

  bno.setExtCrystalUse(true);
  
  digitalWrite(DIRECTION_CONTROL_PINS[0], HIGH);
  digitalWrite(DIRECTION_CONTROL_PINS[1], LOW);
}

void loop() 
{
  Eulers eulers = getEulerAnglesFromQuaternion(bno.getQuat());
  sensors_event_t event;
  bno.getEvent(&event, bno.VECTOR_GRAVITY);

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
      struct PcCommands commands = parseData(data);
      if (commands.command == PUMP_ON_COMMAND || commands.command == PUMP_OFF_COMMAND)
      {
        pumpControl(commands.command, commands.pumpId); 
      }
      else if (commands.command == INIT_BNO)
      {
         init_rpy = getEulerAnglesFromQuaternion(bno.getQuat(), false);
      }
    }
  }
  Serial.print(getEulersMessage(eulers) + getGravityVectorMessage(event) + '\n');
}
