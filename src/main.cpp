#include <Arduino.h>

void setup() {
  Serial.begin(921600);
  Serial.println("Inicio");

}

void loop() {
  delay(2000);
  Serial.println("Hello");
}