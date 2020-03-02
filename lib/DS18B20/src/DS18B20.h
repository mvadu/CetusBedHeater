//Copyright @ Adys Tech
//Author : mvadu@adystech.com

#include <Arduino.h>
#ifndef __DS18B20__
#define __DS18B20__

namespace CommandKeywords
{
const uint8_t Convert_Temp = 0x44;
const uint8_t Read_Scratchpad = 0xBE;
const uint8_t Write_Scratchpad = 0x4E;
const uint8_t Skip_Rom = 0xCC;
}

class DS18B20
{
public:
  enum Resolution
  {
    bit_9 = 0x1F,
    bit_10 = 0x3F,
    bit_11 = 0x5F,
    bit_12 = 0x7F
  };

  DS18B20(uint8_t data_pin, Resolution res = Resolution::bit_12);

  DS18B20(uint8_t power_pin, uint8_t data_pin, Resolution res = Resolution::bit_12);

  ///Return TRUE if sensor is found and can be read. FALSE if DS18B20 is not found.
  bool begin();  
  //Primary read functions
  double getTemperature(void);

private:
  gpio_num_t _data_pin;
  gpio_num_t _vcc_pin;
  Resolution _res;
  unsigned long _lastReadTs;
  double _lastTemp;
  
  bool readBit();
  void writeBit(bool bit);

  bool sendResetPulse(void);
  bool checkCRC(const uint8_t *data, uint8_t len);

  unsigned long getConversionTime();

  uint8_t readByte();
  void writeByte(uint8_t data);

  /**
 * @brief poll the bus until it goes to target state or timeout microseconds.
 *
 *
 * @param  targetState HIGH or LOW
 * @param  timeout no of micro seconds to wait
 * @return
 *     - true if bus went to target state before timeout
 *     - false timeout
 *
 */
  bool waitForBus(int targetState, int timeout);

  /**
    * @brief delay the execution for timeout microseconds, keep polling to catch any state transitions.
    *
    *
    * @param  timeout no of micro seconds to wait
    * @return
    *     - int number of microseconds spent
    *
    */
  int keepBus(int timeout);
};
#endif //__DS18B20__