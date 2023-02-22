#include <Ethernet.h>
#include <SPI.h>
#include <I2CSoilMoistureSensor.h>
#include "Adafruit_seesaw.h"
#include "Wire.h"

#define TCAADDR 0x70

I2CSoilMoistureSensor sensor;

byte command;
byte mac[] = { 0xA8, 0x61, 0x0A, 0xAE, 0x95, 0x36  };
IPAddress ip(192, 168, 1, 13);  // your static IP address
IPAddress serverIP(192, 168, 1, 20);
EthernetClient client;
EthernetServer server(5000);

void tcaselect(uint8_t i) 
{
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

Adafruit_seesaw ss;
float SensorDataBuffer[250];
byte address = 0x36;


void setup() 
{
  Ethernet.begin(mac, ip);
  Serial.begin(9600);
  server.begin();
  sensor.begin(); // reset sensor
  delay(1000); // give some time to boot up
}

void loop() 
{
  EthernetClient client = server.available();
  if (client) {
    if (client.connected()) {
      String request = "";
      while (client.available()) {
        char c = client.read();
        request += c;
        if (c == '\n') {
          break;
        }
      }
      if (request.startsWith("GET ")) {
        String response = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n";
        String data = "{";
  for (uint8_t t=0; t<6; t++) 
        {
          tcaselect(t);
          data+="\"vrstica"+String(t)+"\":{ \"id\":"+String(t);
          for (uint8_t addr = 1; addr<=127; addr++) 
          {
            if (addr == TCAADDR) continue;
            Wire.beginTransmission(addr);
            if (!Wire.endTransmission()) 
            {
              sensor.changeSensor(addr, 1);
              //while (sensor.isBusy()) delay(50);               // available since FW 2.3
              
                
                sensor.changeSensor(addr, 1);
                //Serial.print("here");
                //while (sensor.isBusy()) delay(50);               // available since FW 2.3
                
                float temp = sensor.getTemperature()/(float)10;
                uint16_t cap = sensor.getCapacitance()*2;
                data+=",\"senzor"+String(addr)+"\":{";
                if(addr==59)
                {
                  data+="\"id\":"+String(addr)+",\"temp\":"+String(temp)+",\"cap\":"+String(cap)+"}";
                }
                else
                {
                  data+="\"id\":"+String(addr)+",\"temp\":"+String(temp)+",\"cap\":"+String(cap)+"}";
                }
                
              
            }
          }
          if(t==5){
            data+="}";
          }
          else{
            data+="},";
          }
   
        }
        data+="}";
        response+=data;
        client.print(response);
      }
      client.stop();
    }
  }
}