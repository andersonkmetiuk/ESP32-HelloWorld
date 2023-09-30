/*Project by Anderson Kmetiuk
  Board --> ESP32 WT32 ETH01 
  This is the branch for the Basic Configurations
*/
#include <Arduino.h>

// defines
#define LED1 14 //LED Digital Port 7
#define LED2 15 // LED Digital Port 8

void setup() {
  //Pins Setup
  pinMode(LED1,OUTPUT);
  digitalWrite(LED1, LOW);
  pinMode(LED2,OUTPUT);
  digitalWrite(LED2, LOW);

  Serial.begin(9600);
  delay(5000);
  Serial.println("Serial Begin...");

}

void loop() {
  //HELLO WORLD
  digitalWrite(LED1,HIGH);
  digitalWrite(LED2,LOW);
  Serial.println("LED1 ON - LED2 OFF");
  delay(3000);
  digitalWrite(LED1,LOW);
  digitalWrite(LED2,HIGH);
  Serial.println("LED1 OFF - LED2 ON");
  delay(3000);
}
