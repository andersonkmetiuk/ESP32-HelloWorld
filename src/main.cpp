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
#include <ETH.h>

const char ssid[] = SECRET_SSID;    // your network SSID (name)
const char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

unsigned long loopCount;
#define LOOPSEC 15000

static bool eth_connected = false;

void WiFiEvent(WiFiEvent_t event)
{
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH Started");
      //set eth hostname here
      ETH.setHostname("esp32-ethernet");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.print("ETH MAC: ");
      Serial.print(ETH.macAddress());
      Serial.print(", IPv4: ");
      Serial.print(ETH.localIP());
      if (ETH.fullDuplex()) {
        Serial.print(", FULL_DUPLEX");
      }
      Serial.print(", ");
      Serial.print(ETH.linkSpeed());
      Serial.println("Mbps");
      eth_connected = true;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      eth_connected = false;
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      eth_connected = false;
      break;
    default:
      break;
  }
}

void testClient(const char * host, uint16_t port)
{
  TelnetStream.print("\nconnecting to ");
  TelnetStream.println(host);

  WiFiClient client;
  if (!client.connect(host, port)) {
    TelnetStream.println("connection failed");
    return;
  }
  client.printf("GET / HTTP/1.1\r\nHost: %s\r\n\r\n", host);
  while (client.connected() && !client.available());
  while (client.available()) {
    TelnetStream.write(client.read());
  }

  TelnetStream.println("\n\nclosing connection...\n");
  client.stop();
}

void setup()
{
  Serial.begin(115200);

  //WIFI
  // Serial.print("Attempting to connect to WPA SSID: ");
  // Serial.println(ssid);
  // WiFi.begin(ssid, pass);
  // Serial.println("Connecting");
  // // Wait for WiFi connection
  // while (WiFi.waitForConnectResult() != WL_CONNECTED) {
  //   Serial.println(".");
  //   delay(100);
  // }

  // IPAddress ip = WiFi.localIP();
  // Serial.println();
  // Serial.println("Connected to WiFi network.");
  // Serial.print("Connect with Telnet client to ");
  // Serial.println(ip);


  WiFi.onEvent(WiFiEvent);
  ETH.begin();

  TelnetStream.begin();

  //OTA
  ArduinoOTA.setHostname("ESP32-OTA");
  ArduinoOTA.begin();

  loopCount = millis() + LOOPSEC;
  delay(15000);
  TelnetStream.println("Begin...");
}


void loop()
{
  //OTA Handler
  ArduinoOTA.handle();

  if (millis() > loopCount) {
    TelnetStream.println("\n\n\n\n========================================================\n");
    TelnetStream.print("MAC: ");
    TelnetStream.println(ETH.macAddress());
    TelnetStream.print("IP: ");
    TelnetStream.println(ETH.localIP());
    TelnetStream.print(ETH.linkSpeed());
    TelnetStream.println("Mbps");
    TelnetStream.println("\n========================================================\n\n");
    testClient("google.com", 80);
    loopCount = millis() + LOOPSEC;
  }
}


