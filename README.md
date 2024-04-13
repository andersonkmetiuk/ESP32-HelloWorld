# Wifi-Telnet-OTA

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

#### Telnet

For the Telnet part I have used `Putty`. It's very simple. You just need to paste the `ip` there and select `Telnet` with the default configurations. 

If you don't know the `ip` assigned the best way to figure it out is entering your modem using the default ip for access and the user and password. Usually these informations are written in your modem.

