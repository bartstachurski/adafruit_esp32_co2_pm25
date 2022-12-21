# About

A simple program for ESP32-S3 that regularly checks PM25 and CO2 sensors and sends data to MQTT broker.

I can only only confirm this code works with the following combination of parts:
- [Adafruit PMSA003I Air Quality Breakout - STEMMA QT / Qwiic](https://www.adafruit.com/product/4632)
- [Adafruit QT Py ESP32-S3 WiFi Dev Board with STEMMA QT - 8 MB Flash / No PSRAM](https://www.adafruit.com/product/5426)
- [Adafruit SCD-41 - True CO2 Temperature and Humidity Sensor - STEMMA QT / Qwiic](https://www.adafruit.com/product/5190)
- [STEMMA QT / Qwiic JST SH 4-Pin Cable - 50mm Long](https://www.adafruit.com/product/4399)

# Setup

Copy `secrets.py.template` file, naming the copy `secrets.py`.

Replace the placeholders with the actual secrets.

Save all files to the CIRCUITPYTHON drive.