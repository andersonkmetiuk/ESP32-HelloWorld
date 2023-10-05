/*Project by Anderson Kmetiuk
  Board --> ESP32 WT32 ETH01 
*/
#include <Arduino.h>

// defines
#define LED1 14 // IO14
#define LED2 15 // IO15
#define LED3 12 // IO12
#define LED4 32 // IO32
#define LED5 4 // IO4
#define LED6 2 // IO2

void setup() {
  //Pins Setup
  pinMode(LED1,OUTPUT);
  digitalWrite(LED1, LOW);
  pinMode(LED2,OUTPUT);
  digitalWrite(LED2, LOW);
  pinMode(LED3,OUTPUT);
  digitalWrite(LED3, LOW);
  pinMode(LED4,OUTPUT);
  digitalWrite(LED4, LOW);
  pinMode(LED5,OUTPUT);
  digitalWrite(LED5, LOW);
  pinMode(LED6,OUTPUT);
  digitalWrite(LED6, LOW);

  Serial.begin(9600);
  delay(5000);
  Serial.println("Serial Begin...");

}

void loop() {
  //LED1
  digitalWrite(LED1,HIGH);
  digitalWrite(LED2,LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
  digitalWrite(LED5, LOW);
  digitalWrite(LED6, LOW);
  Serial.println("LED1");
  delay(3000);
  //LED2
  digitalWrite(LED1,LOW);
  digitalWrite(LED2,HIGH);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
  digitalWrite(LED5, LOW);
  digitalWrite(LED6, LOW);
  Serial.println("LED2");
  delay(3000);
  //LED3
  digitalWrite(LED1,LOW);
  digitalWrite(LED2,LOW);
  digitalWrite(LED3, HIGH);
  digitalWrite(LED4, LOW);
  digitalWrite(LED5, LOW);
  digitalWrite(LED6, LOW);
  Serial.println("LED3");
  delay(3000);
  //LED4
  digitalWrite(LED1,LOW);
  digitalWrite(LED2,LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, HIGH);
  digitalWrite(LED5, LOW);
  digitalWrite(LED6, LOW);
  Serial.println("LED4");
  delay(3000);
  //LED3
  digitalWrite(LED1,LOW);
  digitalWrite(LED2,LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
  digitalWrite(LED5, HIGH);
  digitalWrite(LED6, LOW);
  Serial.println("LED5");
  delay(3000);
  //LED3
  digitalWrite(LED1,LOW);
  digitalWrite(LED2,LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
  digitalWrite(LED5, LOW);
  digitalWrite(LED6, HIGH);
  Serial.println("LED6");
  delay(3000);
}
