# Rocket Flight Computer - Divinity

### Project Overview

Divinity is a custom flight computer I developed for our high school rocketry project. I designed its electronics and programmed the firmware to process sensor measurements, estimate the rocket's orientation, and calculate corrections for a thrust vector control (TVC) system.

This project allowed me to combine embedded programming, mathematics, electronics, and control theory to develop an experimental flight-control system.
## How the Flight Computer Works
### Sensor Processing and Orientation Estimation

The flight computer uses an MPU6050 inertial measurement unit (IMU) and a BMP280 barometric pressure sensor.

During initialization, the firmware calibrates the sensors and uses the MPU6050's Digital Motion Processor (DMP) to establish initial orientation offsets.

During operation, accelerometer and gyroscope measurements are processed using the Madgwick orientation filter.

I use quaternions to represent three-dimensional orientation and calculate the difference between the rocket's current direction and its intended vertical direction.
### PID Thrust Vector Control

I implemented a proportional–integral–derivative (PID) controller that uses orientation error and angular velocity measurements to calculate corrective TVC commands.

The controller operates on a scheduled 100 Hz loop and incorporates output limits, integral clamping, and anti-windup logic. Its outputs are converted into commands for two servomotors.
### Altitude Monitoring and Parachute Control

The BMP280 provides barometric altitude estimates. The firmware tracks maximum measured altitude and includes logic to activate a parachute-deployment output when descent conditions are detected.

The program also provides serial diagnostics and includes TFT display functionality, although periodic display updates are disabled in this version.
### System Initialization and Diagnostics

The firmware initializes the sensors, servomotors, and TFT display.

It also provides serial diagnostic information, including orientation vectors and calculated direction errors.

The periodic TFT status-display routine is currently disabled in this version of the code.
## Hardware and PCB Design

I designed a custom PCB for the flight computer. Its KiCad design files are available in the `PCB` folder.

### Differences Between the Firmware and PCB

The firmware and the PCB represents different stages of the project's development. The newer PCB is intended for a future rocket configuration designed to attempt propulsive landing.

- The firmware supports a ST7735 TFT display, while the PCB uses a SD card reader instead.
- The PCB included an additional connection for a second motor igniter, which is not implemented in the provided firmware.
- The firmware contians parachute deployment logic, while the PCB uses a similar connection intended for landing-leg release.

Not all features of the newer PCB are implemented in the uploaded firmware.

## Third-Party Libraries

The firmware uses the following external libraries:

- [MPU6050 / I2Cdevlib](https://github.com/jrowberg/i2cdevlib)
- [Adafruit BMP280](https://github.com/adafruit/Adafruit_BMP280_Library)
- [Adafruit ST7735](https://github.com/adafruit/Adafruit-ST7735-Library)
- [Arduino MadgwickAHRS](https://github.com/arduino-libraries/MadgwickAHRS)
- Arduino Wire
- Arduino Servo

### MadgwickAHRS Modification

I modified the MadgwickAHRS header locally to allow direct access to the filter's beta parameter and quaternion components.

The underlying orientation-estimation algorithm was not modified. The modified library is not included in this repository.

## My Contributions

I developed the flight-computer firmware, including sensor integration, quaternion-based orientation calculations, PID control logic, actuator commands, and recovery-control logic.

I also designed and worked on the custom electronics and PCB.

The rocket's physical airframe was built by another team member. The overall project was developed as part of our school rocketry team with guidance from our teacher.
## Building the Firmware

The firmware was developed for specific hardware using Arduino-compatible libraries.

Reproducing the build requires the appropriate board support package, the listed dependencies, and the locally modified MadgwickAHRS header.

The firmware also contains hardware-specific calibration values and configuration settings.
## Project Status

The firmware requires the appropriate Arduino-compatible board environment, third-party libraries, and hardware-specific configuration.

Divinity is an experimental engineering project developed through iterative programming, electronics design, and testing. Some functions require further validation, and the newer PCB's intended propulsive-landing capabilities have not been demonstrated by the provided firmware.

This repository documents my engineering work, implementation, and design decisions.
