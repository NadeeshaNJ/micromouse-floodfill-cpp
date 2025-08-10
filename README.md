# mms_floodfill_cpp
Micromouse maze-solving algorithm(floodfill) in C++

For use with [mackorone/mms](https://github.com/mackorone/mms), a Micromouse simulator.

This code was initially created to work with an actual [micromouse](https://github.com/NadeeshaNJ/MicroMouse) therefore some of the codes can be redundant for the simulation.
## Info
- `originalmain.cpp` - performs a single Search Run from Start to Goal(center of the maze)<br/>
* `ReturnMain.cpp` - performs a return run where it do a Search Run and a Backtracking. This also shows the shortest path it had discovered.<br/>
+ `DoubleCheckMain.cpp` - performs **two*** Search Runs to refine the map and then shows the final shortest possible path through the discovered cells.<br/>
- `Testing.cpp` is the file I use to test new algorithms.<br/>

# Setup
You can find all the instructions for the simulator in C++ in [here](https://github.com/mackorone/mms-cpp)   

Windows:

1. Clone this repository
2. Download the [Micromouse simulator](https://github.com/mackorone/mms#download)
3. Run the simulator and click the "+" button to configure a new algorithm
4. Enter the config for your algorithm (name, directory, build command, and run command)
5. Click the "Run" button


