//Copyright @ Adys Tech
//Author : mvadu@adystech.com
#include <driver/gpio.h>
#include "DS18B20.h"
#include <Arduino.h>

DS18B20::DS18B20(uint8_t power_pin, uint8_t data_pin, DS18B20::Resolution res, bool usePullup) : DS18B20(data_pin, res, usePullup)
{
    _vcc_pin = (gpio_num_t)power_pin;
    _parasitePower = false;
}

DS18B20::DS18B20(uint8_t data_pin, DS18B20::Resolution res, bool usePullup)
{
    _data_pin = (gpio_num_t)data_pin;
    _res = res;
    mux = portMUX_INITIALIZER_UNLOCKED;
    _parasitePower = true;
    _usePullup = usePullup;
}

bool DS18B20::begin()
{
    if (!_parasitePower)
    {
        gpio_reset_pin(_vcc_pin);
        gpio_pad_select_gpio(_vcc_pin);
        gpio_set_direction(_vcc_pin, gpio_mode_t::GPIO_MODE_OUTPUT);
        gpio_pullup_en(_vcc_pin);
        gpio_set_level(_vcc_pin, 1);
        delay(50);
    }
    //some of the pads on ESP32 are multiplexed to do more than one act. If we want only GPIO then we need to ask.
    gpio_reset_pin(_data_pin);
    gpio_pad_select_gpio(_data_pin);
    gpio_set_direction(_data_pin, gpio_mode_t::GPIO_MODE_INPUT_OUTPUT);
    //gpio_set_drive_capability(_data_pin, GPIO_DRIVE_CAP_1);
    //gpio_set_pull_mode(_data_pin,GPIO_PULLUP_PULLDOWN);
    if (_usePullup)
        gpio_pullup_en(_data_pin);
    //powerdown the bus if no sensor is detected
    if (sendResetPulse())
    {
        setResolution(_res);
        return true;
    }
    else
    {
        gpio_reset_pin(_data_pin);
        gpio_set_level(_data_pin, 0);
        if (_usePullup)
            gpio_pullup_dis(_data_pin);
        if (!_parasitePower)
        {
            gpio_set_level(_vcc_pin, 0);
            gpio_pullup_dis(_vcc_pin);
        }
        Serial.println("Failed with reset");
        return false;
    }
}

bool DS18B20::setResolution(Resolution res)
{
    _res = res;
    if (sendResetPulse())
    {
        writeByte(CommandKeywords::Skip_Rom);
        writeByte(CommandKeywords::Write_Scratchpad);
        delay(10);
        writeByte(0);
        writeByte(0);
        writeByte(_res);
        delay(10);
        return sendResetPulse();
    }
    else
        return false;
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

bool DS18B20::keepBus(int timeout)
{
    if (timeout < 0)
        return 0;
    int initState = gpio_get_level(_data_pin);
    delayMicroseconds(timeout);
    return (gpio_get_level(_data_pin) == initState);
}

unsigned long DS18B20::keepBusForReminder(unsigned long start, int timeout)
{
    unsigned long delta = timeout - (micros() - start);
    if (delta < timeout)
    {
        keepBus(delta);
        return delta;
    }
    return 0;
}

//sends reset pulse and check for presence of the DS18B20 sensor
bool DS18B20::sendResetPulse(void)
{
    //record the init start
    unsigned long t = micros();

    //Tx reset pulse by pulling the 1-Wire bus low for a minimum of 480µs
    gpio_set_level(_data_pin, 0);
    keepBus(480);

    portENTER_CRITICAL(&mux);

    //Let go of the bus, let internal pull up pull it to high
    gpio_set_level(_data_pin, 1);
    waitForBus(HIGH, 15);
    //if (!waitForBus(HIGH, 15))
    //    return false;

    //DS18B20 detects this raising edge, it waits 15µs to 60µs and then transmits a presence pulse
    // by pulling the 1-Wire bus low for 60µs to 240µs.
    keepBusForReminder(t, 16);
    bool presence = waitForBus(LOW, 240);

    portEXIT_CRITICAL(&mux);
    //Serial.printf("presence - %x \n",presence);
    //after presence pulse bus should go back to high, detects shorted pins
    presence = presence && waitForBus(HIGH, 200);
    //total INITIALIZATION PROCEDURE should last 2*480µs minimum
    delay(1);
    return presence;
}

//read one bit from bus, true if the bit is set, else false
bool DS18B20::readBit()
{
    //All read time slots must be a minimum of 60µs in duration with a minimum of a 1µs recovery time between slots.
    bool d;
    portENTER_CRITICAL(&mux);

    //record the read timeslot start
    unsigned long t = micros();
    // initiate a read slot by pulling the 1-Wire bus low for a minimum of 1µs and then releasing the bus
    gpio_set_level(_data_pin, 0);
    waitForBus(LOW, 5);

    //let go of the bus, let it go back to high
    gpio_set_level(_data_pin, 1);
    waitForBus(HIGH, 15);

    //Output data from the DS18B20 is valid for 45µs starting at 15µs after the falling edge that initiated the read time slot.
    //if slave wants to send a 0, it will pull the bus low, else it will be left high, leading to a timeout
    d = !waitForBus(LOW, 45);
    portEXIT_CRITICAL(&mux);

    //wait for remainder of the slot
    keepBusForReminder(t, 100);

    return d;
}

void DS18B20::writeBit(bool bit)
{
    //All write time slots must be a minimum of 60µs in duration with a minimum of a 1µs recovery time
    portENTER_CRITICAL(&mux);

    //record the timeslot start
    unsigned long t = micros();

    // generate a Write time slot by pulling the bus low
    gpio_set_level(_data_pin, 0);
    waitForBus(LOW, 5);

    //to write 1, the master must release the 1-Wire bus within 15µs.
    //low bit is indicated by holding it low
    if (bit == true)
    {
        gpio_set_level(_data_pin, 1);
        waitForBus(HIGH, 15);
    }
    // DS18B20 samples the 1-Wire bus during a window that lasts from 15µs to 60µs
    keepBusForReminder(t, 45);

    //let go of the bus
    gpio_set_level(_data_pin, 1);
    waitForBus(HIGH, 15);

    portEXIT_CRITICAL(&mux);

    //wait for remainder of the slot
    keepBusForReminder(t, 100);
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
    int tCONV = 800;
    switch (_res)
    {
    case Resolution::bit_9:
        return tCONV / 8;
    case Resolution::bit_10:
        return tCONV / 4;
    case Resolution::bit_11:
        return tCONV / 2;
    case Resolution::bit_12:
        return tCONV;
    }
    return 0;
}

double DS18B20::getTemperature(void)
{
    //if there is a read request before the minimum time, return last known value.
    if (!isnan(_lastTemp) && (millis() - _lastReadTs) < getConversionTime())
        return _lastTemp;
    _lastReadTs = millis();
    _lastTemp = readTemperature();
    return _lastTemp;
}

double DS18B20::readTemperature(void)
{
    byte data[9];
    if (sendResetPulse())
    {
        //Serial.printf("Found Sensor \n");
        writeByte(CommandKeywords::Skip_Rom);

        writeByte(CommandKeywords::Convert_Temp);
        //Need to pull bus Up to feed the sensor in parasite power mode (or DS18B20P)
        //strong pullup within 10 μs (max) after a Convert T [44h] or Copy Scratchpad [48h] command is issued
        gpio_set_level(_data_pin, 1);

        //conversion can take upto 750ms
        delay(getConversionTime() + 100);
        uint8_t triesLeft = 3;
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
        } while (triesLeft-- > 0);

        if (triesLeft && data[7] == 0x10)
        {
            //Sometimes parasite powered sensor might get reset midway. detect it by checking power ON reset values
            if (data[0] == 0x50 && data[1] == 0x05)
            {
                setResolution(_res);
                return NAN;
            }

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
            case Resolution::bit_12:
                break;
            }
            // default is 12 bit resolution, 750 ms conversion time
            double t = (double)raw / 16.0;
            //DS18B20 Measures Temperatures from -55°C to +125°C
            if (t > -55 && t < 125)
            {
                if (t == 85.0)
                {
                    Serial.printf("got 85C @ 0 - %x, 1 - %x, 4 - %x, res - %x \n", data[0], data[1], data[4], _res);
                }
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