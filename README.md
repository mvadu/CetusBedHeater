# Cetus Bed Heater

[Cetus](https://www.cetus3d.com) is a nice little 3d printer. By default it does not come with a heated bed. I decided to add an easy to use heater, which I can access over WiFi and set the heater properties. 

This is based on Arduino core running on Esp32 (using [arduino-esp32](https://github.com/espressif/arduino-esp32)) and a simple mosfet to control the heater, and a DS18B20 to measure the bed temperature. The heater itself is formed using a Nichrome 80 wire. The actual web server piece is devlivered by [ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer).

This project uses a self contained WiFI setup portal, so that users can connect to it first, and provide the WiFI credentionals. Post that it uses a bootstrapped UI for the actual heater.

TO server the webpages from ESP32, I developed a powershell script which converts the html into a gzip stream, and stores the stream into a byte array in a header file. This header file can be delivered to GET requests.

I also developed a PID library which can control the PWM on ESP32 with a custom callback function which can provide the current value. 


Ref: 1. [Resistance Wire Heated Build Platform DIY Tutorial](http://airtripper.com/698/resistance-wire-heated-build-platform-diy-tutorial/
)

2. [Arduino PID library](http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/)
