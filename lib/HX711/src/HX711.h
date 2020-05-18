//Copyright @ Adys Tech
//Author : mvadu@adystech.com

#include <Arduino.h>
#ifndef __HX711__
#define __HX711__

class HX711
{
public:
  enum Gain
  {
    //HX711 uses 24 pulses for data, and addition 1-3 pulses for setting gain.
    channel_A_128 = 1,
    channel_B_32 = 2,
    channel_A_64 = 3
  };

  enum SmoothingFactor
  {
    //average the output over x readings
    Low = 8,
    Medium = 16,
    High = 24
  };

  //conversion time depends on the RATE pin and clock used. Most boards use 10Hz.
  HX711(uint8_t sampleRate = 10);

  /**
 * @brief Initialize library with data & clock pins and gain factor. Channel selection is made by passing the appropriate gain:
 *
 * @param  clock_pin SCK pin for HX711
 * @param  data_pin SCK pin for HX711
 * @return
 *     - if sensor is found and can be read.
 *     - if HX711 is not found.
 *
 */
  bool begin(uint8_t clock_pin, uint8_t data_pin, Gain gain = Gain::channel_A_128, SmoothingFactor filter = SmoothingFactor::Medium);
  long getRawReading(bool powerDown=false);
  void tare(bool powerDown=false);

private:
  unsigned long _maxConversionTime;
  gpio_num_t _sda_pin;
  gpio_num_t _sck_pin;
  Gain _gain;
  SmoothingFactor _filter;
  unsigned long _lastReadTs;
  long _lastVal;
  long _tareOffset;
  portMUX_TYPE mux;
  bool _sleeping;
  
  //wake up the sensor
  void wakeup();

  //put the sensor to low power mode
  void sleep();

  //read current reading from sensor
  bool readValue();
  //apply smoothening and return averaged reading
  long getFilteredValue(bool powerDown);
  //check if the sensor is detected and is ready as per datasheet
  bool sensorReady();
  //calculates longest time it should take to read from the sensor
  unsigned long getConversionTime();
  //read one bit from bus, true if the bit is set, else false
  bool readBit();
};
#endif //__HX711__