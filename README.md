# Hephaestus Flight Computer

Embedded flight computer developed for real-time environmental sensing and motion tracking using multiple onboard sensors.

## Overview
Hephaestus is a custom-built flight computer designed to collect and process flight data including altitude, pressure, temperature, and motion. The system integrates multiple sensors to improve data reliability and accuracy through averaging and redundancy.

##  Features
- **Altitude, Temperature, Pressure**
  - Powered by BMP280 barometric sensor
-  **Motion Tracking**
  - Dual MPU6050 (GY-521) IMUs for redundancy
  - Averaged gyro and accelerometer readings for noise reduction
-  **Sensor Fusion Approach**
  - Combines data from multiple IMUs to improve stability and accuracy
  - Complementary filter for Pitch and Roll
-  **Real-Time Telemetry Output**
  - Serial output for debugging and live monitoring
-  **Data Recovery**
  - SD card writing

## System Architecture
- **PCB:** Custom designed Galalumga board
- **Microcontroller:** Teensy 4.1
- **Sensors:**
  - 1x BMP280 (I2C)
  - 2x MPU6050 (I2C, adressed)
- **Data Handling:**
  - Sensor polling loop
  - Averaging algorithm for IMU data
  - Complementary filtered IMU
  - Formatted serial output

## Example Output
<img width="1513" height="234" alt="image" src="https://github.com/user-attachments/assets/afbe94a2-c4d7-4770-84f9-34c6dc938cd3" />


## Tech Stack
- C++ (Arduino framework)
- I2C communication
- Embedded systems programming

## Goals
- Provide reliable flight telemetry
- Reduce sensor noise through redundancy
- Serve as a foundation for future flight logic (e.g., apogee detection, deployment systems)

## Future Improvements
- Kalman filtering / sensor fusion refinement
- Data logging (SD card)
- Wireless telemetry (LoRa / RF)
- Flight event detection (apogee, descent)
- Eventual SMT PCB and sensor upgrades.

## Hardware Setup
<img width="935" height="546" alt="image" src="https://github.com/user-attachments/assets/206de5bb-a208-4b8d-8ecd-56a43a2ca192" />

##  Repository Structure
- /src - main .ino file
- /vis - data visualization algo
- /docs - documentation

## Author
**CR_DGD**

---

### Notes
- Designed for experimentation and iterative development
- Flight count: 0 (pre-flight testing phase)
