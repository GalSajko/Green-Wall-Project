#include <Ethernet.h>
#include <SPI.h>
#include "Adafruit_seesaw.h"
#include "Wire.h"

#define TCAADDR 0x70

byte command;
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 10);  // your static IP address
IPAddress serverIP(192, 168, 1, 20);
EthernetClient client;
void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}
Adafruit_seesaw ss;
float SensorDataBuffer[250];
byte address = 0x36;
void setup() {
  Ethernet.begin(mac, ip);
  Serial.begin(9600);
}
void loop() {
  // prepare data to send
  String data = "{";
  for (uint8_t t=0; t<6; t++) 
        {
          tcaselect(t);
          data+="\"vrstica"+String(t)+"\":{ \"id\":"+String(t);
          for (uint8_t addr = 0; addr<=127; addr++) 
          {
            if (addr == TCAADDR) continue;
            Wire.beginTransmission(addr);
            if (!Wire.endTransmission()) 
            {
              ss.begin(addr);
              float tempC = ss.getTemp();
              uint16_t capread = ss.touchRead(0);
              data+=",\"senzor"+String(addr)+"\":{";
              if(addr==57){
                data+="\"id\":"+String(addr)+",\"temp\":"+String(tempC)+",\"cap\":"+String(capread)+"}";
              }
              else{
                data+="\"id\":"+String(addr)+",\"temp\":"+String(tempC)+",\"cap\":"+String(capread)+"}";
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
  // send HTTP POST request to Flask server
  if (client.connect(serverIP, 5000)) {
    client.println("POST /data HTTP/1.1");
    client.println("Host: your_flask_server.com");
    client.println("Content-Type: application/json");
    client.print("Content-Length: ");
    client.println(data.length());
    client.println();
    client.println(data);
    client.stop();
  }
  delay(2000);
}
