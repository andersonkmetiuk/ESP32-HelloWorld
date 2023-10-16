# Wifi
We have a file `secrets.h` that has only:
```
#define NETWORK_NAME "your_network_name"
#define NETWORK_PASSWORD "your_network_password"
```

```
[env:wt32-eth01]
platform = espressif32@4.2.0
board = wt32-eth01
framework = arduino
```