
#include "ChipInfo.hpp"


    ChipInfo::ChipInfo()
    {
        esp_chip_info_t chip_info;
        esp_chip_info(&chip_info);
        reason = rtc_get_reset_reason(0);
        sdkVersion = ESP.getSdkVersion();
        chipVersion = chip_info.revision;
        //chipVersion = REG_READ(EFUSE_BLK0_RDATA3_REG) >> 15;
        coreCount = chip_info.cores;
        featureBT = (chip_info.features & CHIP_FEATURE_BT) ? 1 : 0;
        featureBLE = (chip_info.features & CHIP_FEATURE_BLE) ? 1 : 0;
        featureWiFi = 1;
        internalFlash = (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? 1 : 0;
        flashSize = spi_flash_get_chip_size() / (1024 * 1024);
    }
