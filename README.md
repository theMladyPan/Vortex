# Rocket Control System
## Overview
This repository contains the implementation of a rocket control system. It is designed to control the orientation and thrust of a rocket using various sensors and actuators. The system is written in C++ and is intended for use with embedded systems, such as those based on Arduino or ESP32 platforms.

## Features
1. Sensor Integration - The system integrates various sensors to collect data necessary for rocket control:
BMI160 Inertial Measurement Unit (IMU): The IMU sensor provides information about the rocket's orientation and angular velocity.
2. Control Components
The control system includes the following components:
PID Regulator: A PID (Proportional-Integral-Derivative) regulator is used to control the rocket's orientation. It calculates corrections based on desired and actual orientation values.
Remote Control: This component allows for remote control of the rocket. It can receive commands and adjust the rocket's orientation and thrust accordingly.
3. Actuators
The system interfaces with servos to control the rocket's orientation and thrust:
Servo Motors: Servo motors are used to adjust the rocket's orientation in the roll and pitch axes. They are also used to control the rocket's throttle.
4. Calibration
The IMU sensor is calibrated during setup to ensure accurate measurements. This calibration process improves the overall performance of the control system.
5. Pre-flight Check
For safety, the system includes a pre-flight check function. This function verifies that all servo motors are functioning correctly before launching the rocket.
6. Real-time Feedback
The system provides real-time feedback through logging. It outputs information about gyro measurements, accelerometer measurements, orientation, desired orientation, corrections, throttle, and servo positions. This feedback is useful for debugging and monitoring the system's performance.

## Usage
To use this control system for your rocket project, follow these steps:
Set up the hardware components, including the IMU sensor and servo motors, and connect them to your microcontroller (ESP32).
Include the necessary header files and source code in your project.
Configure the system parameters, such as PID controller gains, servo pin assignments, and rocket parameters, in your main.cpp file.
Call the appropriate setup functions during the initialization phase.
Implement the main control loop, where you continuously update sensor data, calculate corrections, and adjust servo positions based on the control algorithm.
Monitor the system's performance using the real-time feedback provided through logging.
Ensure that your rocket passes the pre-flight check before launching it.

## Dependencies
This control system depends on the following libraries and components:
Eigen: Eigen is a C++ template library for linear algebra. It is used for matrix and vector operations.
ESP32: If you are using an ESP32 microcontroller, make sure to install the necessary libraries and tools for ESP32 development.
TFT_eSPI (optional): If you have a TFT display connected to your microcontroller, you can use the optional TFT_eSPI library for display functionality.

## License
This control system is provided under the LICENSE file included in the repository. Please review the license before using the code in your project.

## Acknowledgments
We would like to acknowledge the contributions of the open-source community and the developers of the libraries used in this project. Their work has been instrumental in the development of this rocket control system.