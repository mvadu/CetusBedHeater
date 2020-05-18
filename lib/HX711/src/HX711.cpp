//Copyright @ Adys Tech
//Author : mvadu@adystech.com
#include <driver/gpio.h>
#include "HX711.h"
#include <Arduino.h>

HX711::HX711(uint8_t sampleRate)
{
    // conversion time +  27*60µS pulses to read
    _maxConversionTime = 1000 / sampleRate;
    _tareOffset = 0;
    _lastReadTs = 0;
    _lastVal = 0;
    mux = portMUX_INITIALIZER_UNLOCKED;
}

bool HX711::begin(uint8_t clock_pin, uint8_t data_pin, Gain gain, SmoothingFactor filter)
{
    _sda_pin = (gpio_num_t)data_pin;
    _sck_pin = (gpio_num_t)clock_pin;
    _gain = gain;
    _filter = filter;
    //some of the pads on ESP32 are multiplexed to do more than one act. If we want only GPIO then we need to ask.
    gpio_pad_select_gpio(_sda_pin);
    gpio_set_direction(_sda_pin, gpio_mode_t::GPIO_MODE_INPUT);
    gpio_pad_select_gpio(_sck_pin);
    gpio_set_direction(_sck_pin, gpio_mode_t::GPIO_MODE_OUTPUT);

    sleep();
    return !sensorReady();
}

long HX711::getRawReading(bool powerDown)
{
    return getFilteredValue(powerDown) - _tareOffset;
}

long HX711::getFilteredValue(bool powerDown)
{
    if (_lastVal != 0 && (millis() - _lastReadTs) < getConversionTime())
        return _lastVal;

    if (_sleeping)
        wakeup();

    //if the gain is not the default one, read once to setit, but discard the value
    if (_gain != Gain::channel_A_128)
    {
        readValue();
    }

    unsigned long value = 0, start = millis();
    int count = 0;
    while (count < _filter && ((millis() - start) < getConversionTime()))
    {
        if (readValue())
        {
            value += value;
            count++;
        }
    }
    if (powerDown)
        sleep();

    if (count == _filter)
    {
        _lastVal = value / _filter;
        _lastReadTs = millis();
    }
    else
    {
        Serial.printf("failed to get filtered value: %d, %lu", count, value);
    }
    Serial.printf("Total time taken:%lu val: %lu, out:%ld\n", millis() - start, value, _lastVal);
    return _lastVal;
}

void HX711::tare(bool powerDown)
{
    _tareOffset = getFilteredValue(powerDown);
}

void HX711::wakeup()
{
    // Pin PD_SCK input is used to power down the
    // HX711. When PD_SCK Input is low, chip is in
    // normal working mode.
    gpio_set_level(_sck_pin, LOW);
    delayMicroseconds(50);
    //delay(1);
}

void HX711::sleep()
{
    //When PD_SCK pin changes from low to high stays at high
    //for longer than 60µs, HX711 enters power down mode
    gpio_set_level(_sck_pin, LOW);
    delayMicroseconds(50);
    //delay(1);
    gpio_set_level(_sck_pin, HIGH);
    delayMicroseconds(100);
    //delay(1);
    _sleeping = true;
}

bool HX711::readBit()
{
    //raising clock edge triggers the DOUT on sensor
    gpio_set_level(_sck_pin, HIGH);
    delayMicroseconds(1);
    gpio_set_level(_sck_pin, LOW);
    delayMicroseconds(1);
    return gpio_get_level(_sda_pin) == HIGH;
}

bool HX711::readValue()
{
    unsigned long value = 0;
    if (sensorReady())
    {

        portENTER_CRITICAL(&mux);
        for (int i = 0; i < 24; i++)
        {
            if (readBit())
                value |= 0x01 << i;
        }
        portEXIT_CRITICAL(&mux);
        // When input differential signal goes out of the 24 bit range,
        //the output data will be saturated at 800000h (MIN) or 7FFFFFh (MAX),
        if (value == 0x800000 || value == 0x7FFFFF)
        {
            Serial.println("saturated");
            return false;
        }

        //Output data is in 2’s complement fill MSB for -ve numbers
        if (value & 0x800000)
        {
            _lastVal = 0xFF << 24 || value;
        }
        else
        {
            _lastVal = value;
        }

        bool invalid = false;
        //set gain for next reading
        for (int i = 0; i < _gain; i++)
        {
            //The 25th pulse at PD_SCK input will pull DOUT pin back to high
            if (!readBit())
            {
                invalid = true;
            }
            //value += readBit();
        }
        return !invalid;
    }
    return false;
}

bool HX711::sensorReady()
{
    unsigned long start = millis();
    while (millis() - start < _maxConversionTime)
    {
        if (gpio_get_level(_sda_pin) == LOW)
        {
            return true;
        }
        delay(10);
    }
    return false;
}

unsigned long HX711::getConversionTime()
{
    return _filter * 10 > _maxConversionTime ? _filter * 10 : _maxConversionTime;
}