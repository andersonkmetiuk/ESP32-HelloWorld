/* Project by Anderson Kmetiuk
   Board --> ESP32 WT32 ETH01
*/
#include <Arduino.h>
#include <HardwareSerial.h>

#define RXPIN 5
#define TXPIN 17
#define LED1 14 // IO14 pin

unsigned int state = 0; //change LED state
char received = '0';
HardwareSerial SerialPort(2);  //if using UART2

void setup() {
  pinMode(LED1,OUTPUT);
  digitalWrite(LED1, LOW);
  Serial.begin(9600);

  //SerialPort.begin (BaudRate, SerialMode, RX_pin, TX_pin)
  SerialPort.begin(9600, SERIAL_8N1, RXPIN, TXPIN);
  Serial.println("Begin...");
}

void loop() {
  if (SerialPort.available())
  {
     received = SerialPort.read();
    if (received == 'A') {
      state = !state;
      Serial.println("Arduino");
      digitalWrite(LED1, state);
    }
  }
}