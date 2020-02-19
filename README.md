# Cetus Bed Heater

[Cetus](https://www.cetus3d.com) is a nice little 3d printer. By default it does not come with a heated bed. I decided to add an easy to use heater, which I can access over WiFi and set the heater properties. 

This is based on Arduino core running on Esp32 (using [platformio-arduino-esp32](https://docs.platformio.org/en/latest/platforms/espressif32.html)) and a simple mosfet to control the heater, and a DS18B20 to measure the bed temperature. The heater itself is formed using a Nichrome 80 wire. The actual web server piece is delivered by [ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer).

This project uses a self contained WiFI setup portal, so that users can connect to it first, and provide the WiFI credentials. Post that it uses a bootstrapped UI for the actual heater.

There are multiple ways to serve the static webpages from ESP32 via ESPAsyncWebServer. The pages can be saved to SPIFFS or from strings in the code. Another thing to remember is the string needn't be human readable, and it can be a byte stream of gzipped data. This project uses a pre build script which converts the static html into a gzip stream, and stores the stream into a byte array in a header file. This header file can be delivered to GET requests.


Ref: 1. [Resistance Wire Heated Build Platform DIY Tutorial](http://airtripper.com/698/resistance-wire-heated-build-platform-diy-tutorial/
)