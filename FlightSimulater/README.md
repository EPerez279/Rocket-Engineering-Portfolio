# Running the Python Simulator
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
