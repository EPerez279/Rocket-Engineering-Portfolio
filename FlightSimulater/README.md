# Rocket Flight Simulator
## Project Overview

I developed this Python simulator to investigate rocket flight dynamics and control behavior without relying entirely on physical rocket launches.

The simulator uses mathematical models to calculate the rocket's motion and orientation over time, allowing me to experiment with different conditions and control strategies.

Note that this is simulation still experimental, and the landing of the rocket is still in development.
## How It Works

The simulation includes:

- Three-dimensional position, velocity, and orientation represented using quaternions.
- Mathematical models for thrust, gravity, aerodynamic forces, and wind.
- Changes in rocket mass and center of mass during flight.
- PID-based thrust vector control (TVC).
- Fourth-order Runge–Kutta (RK4) numerical integration.
- A modeled second motor ignition at a configurable time.

The simulator produces graphs of flight data and a 3D trajectory plot. Interactive sliders allow changes to rocket mass, wind speed, and second ignition timing.
## Running the Python Simulator
Install the required libraries:
```
pip install numpy matplotlib
```
Run the simulator:
```
python RocketSim.py
```
## Optional C++ Visualizer
The C++ renderer provides a three-dimensional visualization of the simulated rocket's motion. It is disabled by default and requires separate compilation and additional dependencies.
To use the C++ renderer, make `useCPP_Renderer` equal to `true` at the top of the python file.

The flight simulation does not depend on the C++ renderer.
