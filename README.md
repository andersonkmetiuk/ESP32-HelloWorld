# ESP32 - Hello World
Initial development for **ESP32 WT32 ETH01 Board**. 

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
    
## Configuration

Configuring Visual Studio with [PlatformIO Extension](https://platformio.org/install/ide?install=vscode). Remember to set the **platform.ini** file with:
```
[env:wt32-eth01]
platform = espressif32@4.2.0
board = wt32-eth01
framework = arduino
monitor_speed = 921600 
```
To upload your software into the board you will need a USB to UART conversor. For that we will use the **CP2102**.
* **CP2102** &rarr; USB to UART conversor connections:
  * TXD ESP32 &rarr; RXD CP2102
  * RXD ESP32 &rarr; TXD CP2102
  * Remember to connect the **3V3** and **GND** pins as well
  * Note that the **IO0** is connected to the **GND** when you are uploading the code to your flash drive (**Write Mode**).
  * When you are in **Test Mode** you need to disconnect the **IO0** pin.

![cp2102-pins](/assets/images/CP2102-pins.jpeg)

For the version with more pins and modes we use the following configuration:

![cp2102-v2](/assets/images/CP2102-v2.jpg)
