//Copyright @ Adys Tech
//Author : mvadu@adystech.com
#include <driver/gpio.h>
#include "DS18B20.h"
#include <Arduino.h>

DS18B20::DS18B20(uint8_t data_pin, DS18B20::Resolution res)
{
    _data_pin = (gpio_num_t)data_pin;
    _res = res;
    //some of the pads on ESP32 are muliplexed to do more than one act. If we want only GPIO then we need tp ask.
    gpio_pad_select_gpio(_data_pin);
    gpio_set_direction(_data_pin, gpio_mode_t::GPIO_MODE_INPUT_OUTPUT);
    gpio_pullup_en(_data_pin);
    //powerdown the bus if no sensor is detected
    if (sendResetPulse())
    {
        writeByte(CommandKeywords::Skip_Rom);
        writeByte(CommandKeywords::Write_Scratchpad);
        delay(10);
        writeByte(0);
        writeByte(0);
        writeByte(res);
        delay(10);
        sendResetPulse();
        Serial.println("Sensor found!");
    }
    else
    {
        gpio_pullup_dis(_data_pin);
        Serial.println("Sensor not found!");
    }
}

bool DS18B20::waitForBus(int targetState, int timeout)
{
    while (gpio_get_level(_data_pin) != targetState)
    {
        if (--timeout == 0)
            break;
        delayMicroseconds(1);
    };
    return gpio_get_level(_data_pin) == targetState;
}

int DS18B20::keepBus(int timeout)
{
    unsigned long t = micros();
    int initState = gpio_get_level(_data_pin);
    if (timeout < 0)
        return 0;
    do
    {
        if (--timeout == 0)
            break;
        delayMicroseconds(1);
    } while (gpio_get_level(_data_pin) == initState);
    return micros() - t;
}

//sends reset pulse and check for presence of the DS18B20 sensor
bool DS18B20::sendResetPulse(void)
{
    //Tx reset pulse by pulling the 1-Wire bus low for a minimum of 480µs
    gpio_set_level(_data_pin, 0);
    keepBus(480);

    //Let go of the bus, let internal pull up pull it to high
    gpio_set_level(_data_pin, 1);
    waitForBus(HIGH, 15);

    //DS18B20 detects this rising edge, it waits 15µs to 60µs and then transmits a presence pulse
    // by pulling the 1-Wire bus low for 60µs to 240µs.
    keepBus(15);
    bool presence = waitForBus(LOW, 240);

    //after presence pulse bus should go back to high, detects shorted pins
    presence = presence && waitForBus(HIGH, 200);
    keepBus(100);
    return presence;
}

//read one bit from bus, true if the bit is set, else false
bool DS18B20::readBit()
{
    //All read time slots must be a minimum of 60µs in duration with a minimum of a 1µs recovery time between slots.
    unsigned long t;
    bool d;

    //record the read timeslot start
    t = micros();
    // initiate a read slot by pulling the 1-Wire bus low for a minimum of 1µs and then releasing the bus
    gpio_set_level(_data_pin, 0);
    keepBus(5);

    //let go of the bus, let it go back to high
    gpio_set_level(_data_pin, 1);
    keepBus(5);

    //Output data from the DS18B20 is valid for 15µs after the falling edge that initiated the read time slot.
    //if slave wants to send a 0, it will pull the bus low, else it will be left high
    d = gpio_get_level(_data_pin) == HIGH;

    //wait for remainder of the slot
    keepBus(50 - (micros() - t));
    return d;
}

void DS18B20::writeBit(bool bit)
{
    //All write time slots must be a minimum of 60µs in duration with a minimum of a 1µs recovery time
    unsigned long t;
    //record the timeslot start
    t = micros();

    // generate a Write time slot by pulling the bus low
    gpio_set_level(_data_pin, 0);
    //to write 1, the master must release the 1-Wire bus within 15µs.
    //low bit is indicated by holding it low
    if (bit == true)
    {
        keepBus(10);
        gpio_set_level(_data_pin, 1);
    }
    // DS18B20 samples the 1-Wire bus during a window that lasts from 15µs to 60µs
    keepBus(55);
    //let go of the bus
    gpio_set_level(_data_pin, 1);
    //wait for remainder of the slot
    keepBus(60 - (micros() - t));
}

uint8_t DS18B20::readByte()
{
    uint8_t data = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        if (readBit())
            data |= 0x01 << i;
    }
    return data;
}

void DS18B20::writeByte(uint8_t data)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        writeBit((data >> i & 0x01) == 0x01);
    }
}
unsigned long DS18B20::getConversionTime()
{
    switch (_res)
    {
    case Resolution::bit_9:
        return (94 + 10);
    case Resolution::bit_10:
        return (188 + 20);
    case Resolution::bit_11:
        return (375 + 40);
    case Resolution::bit_12:
        return (750 + 100);
    }
}
double DS18B20::getTemperature(void)
{
    //if there is a read request before the minimum time, return last known value.
    if (!isnan(_lastTemp) && (millis() - _lastReadTs) < getConversionTime())
        return _lastTemp;

    byte data[9];
    if (sendResetPulse())
    {
        //Serial.printf("Found Sensor \n");
        writeByte(CommandKeywords::Skip_Rom);
        writeByte(CommandKeywords::Convert_Temp);
        //conversion can take upto 750ms
        delay(getConversionTime());
        uint8_t triesLeft = 10;
        do
        {
            sendResetPulse();
            writeByte(CommandKeywords::Skip_Rom);
            writeByte(CommandKeywords::Read_Scratchpad);
            //Serial.print("Scratchpad: ");
            for (uint8_t i = 0; i < 9; i++)
            {
                data[i] = readByte();
                //    Serial.print(data[i], HEX);
            }
            //Serial.println("");
            if (checkCRC(data, 9))
                break;
            delay(10);
        } while (triesLeft-- > 0);

        if (triesLeft && data[7] == 0x10)
        {
            int16_t raw = (data[1] << 8) | data[0];
            // at lower res, the low bits are undefined, so let's zero them
            switch (_res)
            {
            case Resolution::bit_9:
                raw = raw & ~7;
                break;
            case Resolution::bit_10:
                raw = raw & ~3;
                break;
            case Resolution::bit_11:
                raw = raw & ~1;
                break;
            }
            // default is 12 bit resolution, 750 ms conversion time
            double t = (double)raw / 16.0;
            //DS18B20 Measures Temperatures from -55°C to +125°C
            if (t > -55 && t < 125)
            {
                _lastReadTs = millis();
                _lastTemp = t;
                return t;
            }
            else
            {
                //Serial.println("invalid temp");
                return NAN;
            }
        }
        //Serial.println("read failed");
        return NAN;
    }
    else
    {
        //Serial.println("reset failed");
        return NAN;
    }
}

//APPLICATION NOTE 27 Understanding and Using Cyclic Redundancy Checks with Maxim 1-
//https://www.microchip.com/webdoc/AVRLibcReferenceManual/group__util__crc_1ga37b2f691ebbd917e36e40b096f78d996.html
bool DS18B20::checkCRC(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; ++i)
    {
        crc = crc ^ data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x01)
                crc = (crc >> 1) ^ 0x8C; //0x31 right to left reversed
            else
                crc >>= 1;
        }
    }
    //If a correct read has occurred, the shift register is again all 0s
    return crc == 0;
}