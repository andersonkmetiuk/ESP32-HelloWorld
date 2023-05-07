/* Project by Anderson Kmetiuk
   Board --> ESP32 WT32 ETH01
   The main purpose of this repo is to test the basic features of the board
   and try simple projects just to get use to
*/
#include <Arduino.h>

void setup() {
  Serial.begin(921600);
  Serial.println("Inicio");

}

void loop() {
  delay(2000);
  Serial.println("Hello");
}