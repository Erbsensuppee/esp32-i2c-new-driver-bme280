# Home Meter - Environmental Monitoring with ESP32 and BME280

## Overview
The Home Meter project is an ESP32-based environmental monitoring system that uses the Bosch BME280 sensor to measure:
- **Temperature**
- **Pressure**
- **Humidity**

This implementation is written in C using the ESP-IDF framework and serves as an example of I2C communication and FreeRTOS task management.

## Features
- Configurable I2C interface with internal pull-up resistors.
- Periodic sensor data readings with 1-second intervals.
- Simple and clear structure for future expansion.

## Getting Started
1. Clone this repository.
2. Install ESP-IDF as per [official instructions](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/).
3. Configure the project for your ESP32 device.
4. Build and flash the firmware:
   ```bash
   idf.py build flash monitor
