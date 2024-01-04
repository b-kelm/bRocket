# Flight-Logger for a super-small Spin-stabilized Rocket 

Author: Benjamin Kelm
Date: 04.01.2024 
Name: bRocket - baby Rocket (super small)


## Description and Features
This software is a basic, high-frequency flight logger for rockets. 

**Code functions:**
1. Read IMU (Gyro + Accel), Barometer (and Thermometer) values from **GY-87** (MPU6050 + BMP180) Sensor  
2. Save Sensor Readings on micro SD-card (binary encoding)
3. Decode flight logs, visualize them and export them to CSV

**Features**
- Very small form factor 
- 300 [Hz] Accelerometer and Gyroscope Logging
- ≈ 60 [Hz] Barometer Logging
- Open Source (Arduino/Teensyduino + Python)
- (almost) unlimited recording time

**Note:** This project is *educational*. Do not expect a flawless flight logger.

## Electronics
Target Board: Teensy 4.1 
Environment: [Teensyduino](https://www.pjrc.com/teensy/td_download.html) 

### Components Used:

1. [Teensy 4.1](https://www.pjrc.com/store/teensy41.html) with micro-SD card inserted
2. [MPU6050 + BMP180 (GY-87)](https://www.christians-shop.de/GY-87-10DOF-MPU6050-HMC5883L-BMP180-Drei-Achsen-Gyroskop-dreiachsiger-Beschleunigungsmesser-Drei-Achsen-Magnetfeld-Atmosphaerendruck)

### Wiring:
Very simple: 
* 1S Lipo to 5V and GND (Teensy)
* GY-87 I2C (SCL, SDA) to Teensy I2C
* GY-87 3.3V and GND to Teensy

## Program Description
The program is heavily relying on these sources:
- [teensy SD Logger](https://forum.pjrc.com/threads/66165-Minimalistic-SdFat-Datalogger-for-Teensy4-1-Example) by MBorgerson
For all other details, see the comments in the code.

## Disclaimer
This program is free software. It comes without any warranty, to the extent permitted by applicable law. 
You can redistribute it and/or modify it under the terms of the [MIT License](https://github.com/git/git-scm.com/blob/main/MIT-LICENSE.txt), as published by Scott Chacon and others.
