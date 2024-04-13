/* Project by Anderson Kmetiuk
   Board --> ESP32 WT32 ETH01
   The main purpose of this repo is to test the basic features of the board
   and try simple projects just to get use to
*/
#include <Arduino.h>

#include <secrets.h> //passwords - This file is in the .gitignore
//just create a file with #define and the value of password and network name

#include <ArduinoOTA.h>
#include <WiFi.h>
#include <TelnetStream.h>

#define LED 14
#define LEDTIME 500

const char ssid[] = SECRET_SSID;    // your network SSID (name)
const char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

void setup() {
  Serial.begin(115200);

  //WIFI
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);
  // Wait for WiFi connection
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Failed to connect.");
    while (1) {
      delay(10);
    }
  }

  IPAddress ip = WiFi.localIP();
  Serial.println();
  Serial.println("Connected to WiFi network.");
  Serial.print("Connect with Telnet client to ");
  Serial.println(ip);

  TelnetStream.begin();

  //OTA
  ArduinoOTA.setHostname("ESP32-OTA");
  ArduinoOTA.begin();

  // LED Setup
  pinMode(LED,OUTPUT);
  digitalWrite(LED, LOW);
}

void loop() {
  //OTA Handler
  ArduinoOTA.handle();

  switch (TelnetStream.read()) {
    case 'R':
    TelnetStream.stop();
    delay(100);
    ESP.restart();
      break;
    case 'C':
      TelnetStream.println("bye bye");
      TelnetStream.stop();
      break;
  }

  //Simple Test
  TelnetStream.println("LED ON");
  digitalWrite(LED, HIGH);
  delay(LEDTIME);
  TelnetStream.println("LED OFF");
  TelnetStream.println("--------");
  digitalWrite(LED,LOW);
  delay(LEDTIME);
}


