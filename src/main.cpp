/*Project by Anderson Kmetiuk
  Board --> ESP32 WT32 ETH01 
  This is the branch for the Basic Configurations
  It's for configuring the basics for the Board
  The board that we are using is the Esp32 WT32 ETH01
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