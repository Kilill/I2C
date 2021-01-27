# ESP8266 ESP_RTOS_SDK and ESP32 ESP_IDF CPP class for I2C
[![Framework Badge ESP8266]()https://img.shields.io/static/v1?label=Platform&message=ESP2866%20SDK&color=blue](https://github.com/espressif/ESP8266_RTOS_SDK)

Based on version by https://github.com/UncleRus

## Example
```cpp
#include "I2C.h"

const uint8_t address = 0x4D;
const uint8_t SDA = 2;
const uint8_t SCL = 14;
const uint8_t I2CPORT = 0;

MCP3021 mcp3021(address);
I2C bus(I2CPORT,SDA,SCL);

bus.begin();
bus.writeByte(0xff);
bus.end();
```
