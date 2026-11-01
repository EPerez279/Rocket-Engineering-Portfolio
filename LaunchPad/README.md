# Genesis - Launch Pad Control Computer

## Project Overview

Genesis is an ESP32-based launch-pad control computer I developed for our high school rocketry project. I designed the electronics and developed the firmware and wireless web interface to manage the launch-pad mechanisms.

The project combines embedded programming, wireless communication, web development, and electromechanical control.
## How It Works

The ESP32 creates a Wi-Fi access point and hosts a browser-based control interface written in HTML, CSS, and JavaScript.

The firmware includes:

- A countdown and launch-status interface.
- A state machine for managing different stages of the launch sequence.
- Control of four servomotors associated with the launch-pad locking and release mechanism.
- Web-server endpoints for interacting with the controller.
- An ignition-control output.

The interface communicates with the ESP32 through HTTP requests.
## Hardware and Software

### Hardware

I designed the custom Genesis PCB, integrating the ESP32 controller, four servo connections, and a rocket motor igniter connection. The schematic and PCB layout are available in the `PCB` directory.

- Custom launch-pad PCB
- Arduino Nano ESP32
- Four servo connections for the launch-pad
  locking and release mechanism
- Rocket motor igniter connection

### Software and Libraries
- [Arduino-ESP32 Core](https://github.com/espressif/arduino-esp32)
  - WiFi
  - WebServer
  - Wire
- [ESP32Servo](https://github.com/madhephaestus/ESP32Servo)
- [Bootstrap 5.3](https://getbootstrap.com/docs/5.3/)
- HTML, CSS, and JavaScript

## Development and Testing

Genesis was developed and tested on the custom launch-pad PCB and used to operate the launch-pad system during some of our model-rocket launches.

Not every launch shown in the portfolio used Genesis.

The repository contains the original firmware from development and testing. It also includes experimental interface elements, such as a commented-out 3D visualization and unfinished telemetry displays.
## My Contributions

I designed the launch-pad electronics and developed the ESP32 firmware and browser interface.

My work included integrating wireless communication, programming the control system, and developing the interface for interacting with the launch-pad mechanisms.

The project was developed as part of our school rocketry team with teacher guidance.
## Project Status

Genesis is a tested experimental prototype, not a fully safety-validated launch system. The source code documents its implemented functionality and development process.
