#include <Arduino.h>
#include "soc/efuse_reg.h"
#include <esp_spi_flash.h>
#include <esp_system.h>
#include <rom/rtc.h>

class ChipInfo
{
public:
  ChipInfo();
  uint8_t reason;
  const char *sdkVersion;
  uint8_t chipVersion;
  uint8_t coreCount;
  uint8_t featureBT;
  uint8_t featureBLE;
  uint8_t featureWiFi;
  bool internalFlash;
  uint8_t flashSize;
};