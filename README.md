# ESP32 - Hello World
Initial development for `ESP32 WT32 ETH01` Board. 

![Esp32-pinout](/assets/images/wt32-eth01-pinout.png)

## Specifications:

- Supply voltage: 3.3V DC (via 3V3 pin) or 5V DC (via 5V pin)
- GPIO Voltage: 3.3V
- Microcontroller:
  - Chip: ESP32-D0WD-Q6 (WT32-S1)
  - Clock Frequency: 240MHz
  - Processor cores: 2
  - SRAM: 520KB
  - Flash memory: 4MB
  - Built-in Wi-Fi
  - Built-in Bluetooth
  - Number of GPIO pins: 15
  - Number of PWM pins: 12
  - Number of analog input pins (ADC): 11
  - Number of analog output pins (DAC): 0
- Ethernet:
  - Chip: LAN8720A
  - Transfer rate: 10/100Mbps
  - Connector: RJ45
- USB to serial converter: None (use an external one)

![WT32-ETH01-specs](/assets/images/wt32-specs.png)

You can check the [documentation folder](https://github.com/andersonkmetiuk/ESP32-HelloWorld/tree/main/documentation) for more information.
    
## Configuration

Configuring Visual Studio with [PlatformIO Extension](https://platformio.org/install/ide?install=vscode). Remember to set the `platform.ini` file with:
```
[env:wt32-eth01]
platform = espressif32@4.2.0
board = wt32-eth01
framework = arduino
```
If you have another model of `ESP32` you cand find the supported list [here](https://registry.platformio.org/platforms/platformio/espressif32/boards)

[PlatformIO Configuration](https://docs.platformio.org/en/latest/boards/espressif32/wt32-eth01.html)

For Linux you might need to run this command
```
sudo apt-get install python3-venv
```
[Here's why](https://github.com/platformio/platformio-core-installer/issues/1774)

You can check [here](https://randomnerdtutorials.com/vs-code-platformio-ide-esp32-esp8266-arduino/) if you want a step by step tutorial.

---
## Software Burn with CP2102 (USB - UART module)
To upload your software into the board you will need a USB to UART conversor. For that we will use the `CP2102`.

* **CP2102** &rarr; USB to UART conversor connections:
  * TXD ESP32 &rarr; RXD CP2102
  * RXD ESP32 &rarr; TXD CP2102
  * Remember to connect the **3V3** and **GND** pins as well
  * Note that the **IO0** is connected to the **GND** when you are uploading the code to your flash drive (**Write Mode**).
  * When you are in **Test Mode** you need to disconnect the **IO0** pin.

![cp2102-pins](/assets/images/CP2102-pins.jpeg)

For the version with more pins and modes we use the following configuration:

![cp2102-v2](/assets/images/CP2102-v2.jpg)

## Pin Reference in Arduino.h
[Source](https://github.com/espressif/arduino-esp32/blob/master/variants/wt32-eth01/pins_arduino.h)
```
#ifndef Pins_Arduino_h
#define Pins_Arduino_h

/**
 * Variant: WT32-ETH01
 * Vendor: Wireless-Tag
 * Url: http://www.wireless-tag.com/portfolio/wt32-eth01/
 */

#include <stdint.h>

#define EXTERNAL_NUM_INTERRUPTS 16
#define NUM_DIGITAL_PINS 40
#define NUM_ANALOG_INPUTS 16

#define analogInputToDigitalPin(p) (((p) < 20) ? (analogChannelToDigitalPin(p)) : -1)
#define digitalPinToInterrupt(p) (((p) < 40) ? (p) : -1)
#define digitalPinHasPWM(p) (p < 34)

// interface to Ethernet PHY (LAN8720A)
#define ETH_PHY_ADDR 1
#define ETH_PHY_POWER 16
#define ETH_PHY_MDC 23
#define ETH_PHY_MDIO 18
#define ETH_PHY_TYPE ETH_PHY_LAN8720
#define ETH_CLK_MODE ETH_CLOCK_GPIO0_IN

// general purpose IO pins
static const uint8_t IO0 = 0;
static const uint8_t IO1 = 1; // TXD0 / TX0 pin
static const uint8_t IO2 = 2;
static const uint8_t IO3 = 3; // RXD0 / RX0 pin
static const uint8_t IO4 = 4;
static const uint8_t IO5 = 5; // RXD2 / RXD pin
static const uint8_t IO12 = 12;
static const uint8_t IO14 = 14;
static const uint8_t IO15 = 15;
static const uint8_t IO17 = 17; // TXD2 / TXD pin
static const uint8_t IO32 = 32; // CFG pin
static const uint8_t IO33 = 33; // 485_EN pin

// input-only pins
static const uint8_t IO35 = 35;
static const uint8_t IO36 = 36;
static const uint8_t IO39 = 39;

// UART interfaces
static const uint8_t TXD0 = 1, TX0 = 1;
static const uint8_t RXD0 = 3, RX0 = 3;
static const uint8_t TXD2 = 17, TXD = 17;
static const uint8_t RXD2 = 5, RXD = 5;
static const uint8_t TX = 1;
static const uint8_t RX = 3;

//SPI VSPI default pins
static const uint8_t SS    = -1;
static const uint8_t MOSI  = 14;
static const uint8_t MISO  = 15;
static const uint8_t SCK   = 12;

//I2C default pins
static const uint8_t SDA = 33;
static const uint8_t SCL = 32;

#endif /* Pins_Arduino_h */
```
## Examples

### Branch: Basic-Configurations
```
#include <Arduino.h>

// defines
#define LED1 14 // IO14 pin
#define LED2 15 // IO15 pin

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
```

---

### Branch: 6-LED
```
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
```

---

### Branch: Millis-Blink 
```
#include <Arduino.h>

#define LED1 7 //LED Digital Port 7
#define BLINK 75

unsigned long blink_timer;
unsigned int led_state;
void setup() {
  //Pins Setup
  pinMode(LED1,OUTPUT);
  digitalWrite(LED1, LOW);
  blink_timer = millis() + BLINK;
  led_state = 1;
}

void loop() {
  //HELLO WORLD
  if(millis() > blink_timer)
  {
    digitalWrite(LED1, led_state);
    led_state = !led_state;
    blink_timer = millis() + BLINK;
  }
}
```

---

### Branch: Button-Blinky
```
#include <Arduino.h>
//Press the Button to turn both LEDs ON/OFF

// defines
#define LED1 14 // IO14
#define LED2 15 // IO15
#define BUTTON 39 //Input Only Pins - IO35, IO36 or IO39

//global var
unsigned int state1 = 1,state2 = 0; //change LED state
unsigned int pressB = 0; //button presses


void setup() {
  //Pins Setup
  pinMode(LED1,OUTPUT);
  digitalWrite(LED1, HIGH);
  pinMode(LED2,OUTPUT);
  digitalWrite(LED2, LOW);
  pinMode(BUTTON, INPUT);
  Serial.begin(9600);
  Serial.println("Begin...");
}

void loop() {
  //check if the button is pressed then change the state of the LEDs to ON/OFF
  if(digitalRead(BUTTON))
  {
    state1 = !state1;
    state2 = !state2;
    Serial.println("Button");
    digitalWrite(LED1,state1);
    digitalWrite(LED2,state2);
    pressB++; //increment button press
    Serial.println(pressB);
    delay(100); //debounce
  }
  delay(100); //debounce
}
```

---

### Branch: Communication-Arduino
Let's try to send a message with an `Arduino Nano` and then get a response and blink a LED with the `ESP32`. I have used this [guide](https://microcontrollerslab.com/esp32-uart-communication-pins-example/) as a reference.

#### Arduino Nano
There are a few steps for the Arduino. If you want just a quick test you can use [this](https://github.com/andersonkmetiuk/ArduinoNano-HelloWorld/tree/Communication-ESP32) as a reference and burn it into your `Arduino Nano`.

If you want to do it yourself. Remember to set the `platform.ini` like so
```
[env:nanoatmega328]
platform = atmelavr
board = nanoatmega328new
framework = arduino
```
We are going to use a simple loop just to send the letter 'A' through the UART
```
#include <Arduino.h>

void setup() {
  Serial.begin(9600);
  Serial.println("Begin...");
}

void loop() {
 Serial.println("A"); //Sends data to the ESP32 through UART
 delay(5000);
}
```

#### ESP32
For the `ESP32` we are going to use the library `HardwareSerial` to set the second UART to receive the data like so

```
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
```

---

### Branch: Bluetooth
This is the [guide](https://randomnerdtutorials.com/esp32-bluetooth-classic-arduino-ide/) I have followed to set up the Bluetooth.
```
#include <Arduino.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
}

void loop() {
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }
  delay(20);
}
```

---

### Branch: Wifi
```
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
 Serial.print("IP: ");
 Serial.println(WiFi.localIP());
}
 
void loop() {}
```

You need to have a file `secrets.h` that has only
```
#define NETWORK_NAME "your_network_name"
#define NETWORK_PASSWORD "your_network_password"
```

---

### Branch: Ethernet
Basic test example for ethernet
```
#include <Arduino.h>
#include <Update.h>
#include <ETH.h>

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
  Serial.print("\nconnecting to ");
  Serial.println(host);

  WiFiClient client;
  if (!client.connect(host, port)) {
    Serial.println("connection failed");
    return;
  }
  client.printf("GET / HTTP/1.1\r\nHost: %s\r\n\r\n", host);
  while (client.connected() && !client.available());
  while (client.available()) {
    Serial.write(client.read());
  }

  Serial.println("closing connection\n");
  client.stop();
}

void setup()
{
  Serial.begin(115200);
  WiFi.onEvent(WiFiEvent);
  ETH.begin();
}


void loop()
{
  if (eth_connected) {
    testClient("google.com", 80);
  }
  delay(10000);
}
```

---

### Branch: Wifi-Telnet-OTA

At first I had issues while connecting to the WiFi.

Error: `Brownout detector was triggered`

This means that your USB port is not providing enough power. The solution for that is connecting an external source and a `1000uF capacitor` between `Vcc` and `GND`.

As I am using `PlatformIO` this is how the `platform.ini` looks like:

```
;basics
[env:wt32-eth01]
platform = espressif32@4.2.0
board = wt32-eth01
framework = arduino

;advanced
monitor_speed = 115200

;For more on upload protocol check https://docs.platformio.org/en/latest/boards/espressif32/wt32-eth01.html
upload_protocol = espota
upload_port = 192.168.0.148
;upload_port = COM4
;monitor_port = COM4
build_src_filter = +<main.cpp>

;For more check out https://docs.platformio.org/en/latest/projectconf/sections/env/options/library/lib_deps.html
lib_deps =
  jandrassy/TelnetStream@1.2.5
  jandrassy/NetApiHelpers@1.0.0
  ArduinoOTA@2.0.0
  WiFi@2.0.0
```

#### Be mindful

The first time you send this code to the ESP32 you need to comment the line `upload_protocol` and the `upload_port` using a `;`. And if you want to keep using OTA updates remember that you need to keep OTA configurations inside of your source code every time you upload a new code through OTA using the IP. If you don't do that you need to reupload a code with the OTA configurations.

You need to have a file `secrets.h` that has only
```
#define SECRET_SSID "your_network_name"
#define SECRET_PASS "your_network_password"
```

This is the main code

```
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
```

#### Telnet

For the Telnet part I have used `Putty`. It's very simple. You just need to paste the `ip` there and select `Telnet` with the default configurations. 

If you don't know the `ip` assigned the best way to figure it out is entering your modem using the default ip for access and the user and password. Usually these informations are written in your modem.

---