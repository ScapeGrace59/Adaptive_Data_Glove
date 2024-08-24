# Adaptive_Data_Glove

This project aims to improve the efficiency and accuracy of low-tier filters in data gloves by implementing advanced optimization techniques. Specifically, it integrates the Hooke and Jeeves algorithm with the Mahony filter for enhanced orientation tracking and utilizes Reinforcement Learning (RL) to calibrate flex sensors for precise finger movement detection. These improvements are designed to make data gloves more effective and reliable for use in Virtual Reality (VR) and telerobotic applications, even with low-cost hardware.

# Hardware:
- **IMU Sensor:** MPU-6050
- **Flex Sensors:** Standard low-cost flex sensors
- **Microcontroller:** ESP32-WROOM-DEVKITC
# Software:
- **Microcontroller Programming:** Arduino (C++)
- **Simulation and Testing:** Unity (C#)
  - **Unity Version:** 2019.4.21f1

This project makes use of the modified versions of the following repositories:
- **I2Cdevlib**: [I2Cdevlib by jrowberg](https://github.com/jrowberg/i2cdevlib) - A collection of I2C device drivers for Arduino, including support for the MPU-6050 sensor for DMP access.
- **Mahony Filter**: [MPU-6050 Fusion by jremington](https://github.com/jremington/MPU-6050-Fusion) - An implemantion of complimentary filter for MPU-6050.

Please carefully check the pin numbers in the code before uploading it to the microcontroller to ensure proper configuration and operation.
