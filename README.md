# ESP32HelloWorld
Initial development for **ESP32 WT32 ETH01 Board**. 

Configuring Visual Studio with **PlatformIO** Extension. Remember to set the **platform.ini** file with:
```
[env:wt32-eth01]
platform = espressif32@4.2.0
board = wt32-eth01
framework = arduino
monitor_speed = 921600 
```

* **CP2102** &rarr; USB to UART conversor connections:
  * TXD ESP32 &rarr; RXD CP2102
  * RXD ESP32 &rarr; TXD CP2102
  * Remember to connect the **3V3** and **GND** pins as well
  * Note that the **IO0** is connected to the **GND** when you are uploading the code to your flash drive.

![cp2102-pins](/assets/images/CP2102-pins.jpeg)

