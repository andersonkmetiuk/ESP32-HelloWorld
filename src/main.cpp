/* Project by Anderson Kmetiuk
   Board --> ESP32 WT32 ETH01
   The main purpose of this repo is to test the basic features of the board
   and try simple projects just to get use to
*/
#include <Arduino.h>
#include <WiFi.h>
#include <Update.h>
#include "secrets.h"

const char* ssid = NETWORK_NAME;
const char* password =  NETWORK_PASSWORD;

void setup() {
 Serial.begin(115200);
 WiFi.begin(ssid, password);
 
 while (WiFi.status() != WL_CONNECTED) {
  delay(500);
  Serial.println("Connecting to WiFi..");
 }
 Serial.println("Connected to the WiFi network");
 Serial.println("Endereço de IP: ");
 Serial.println(WiFi.localIP());
}
 
void loop() {}