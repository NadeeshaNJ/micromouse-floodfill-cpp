# Micromouse Floodfill Maze Solver in C++ for MMS Simulator
Micromouse maze-solving algorithm(floodfill) in C++

This project implements a **Micromouse floodfill maze-solving algorithm** in **C++**, designed to run with the [mms Micromouse Simulator](https://github.com/mackorone/mms).  
It supports **search runs**, **Backtracking**, and **shortest path discovery**, making it ideal for robotics competitions and micromouse algorithm research.

This code was originally developed for an actual [micromouse](https://github.com/NadeeshaNJ/MicroMouse), so some parts may be redundant for the simulation.
## Info
This repository contains multiple versions of the main control loop for different micromouse run strategies:  
- `Floodfill_SearchRun.cpp` - performs a single Search Run from Start to Goal(center of the maze)<br/>
* `Floodfill_Backtracking.cpp` - performs a return run where it do a Search Run and a Backtracking. This also shows the shortest path it had discovered.<br/>
+ `Floodfill_DoubleSearch.cpp` - performs **two*** Search Runs to refine the map and then shows the final shortest possible path through the discovered cells.<br/>
- `Testing.cpp` is the file I use to test new algorithms.<br/>


![Micromouse maze solving using floodfill algorithm in mms simulator](https://github.com/NadeeshaNJ/micromouse-floodfill-cpp/blob/main/Maze-solving-using-floodfill.png)
## Setup

You can find all the instructions for the simulator in C++ from [here](https://github.com/mackorone/mms-cpp)   

Windows:

1. Clone this repository
2. Download the [Micromouse simulator](https://github.com/mackorone/mms#download)
3. Run the simulator and click the "+" button to configure a new algorithm
4. Enter the config for your algorithm (name, directory, build command, and run command)
5. Click the "Run" button

## Additional Resources

You can find mazefiles from [here](https://github.com/micromouseonline/mazefiles) . This includes mazes from international level competitions<br/>
There is a very useful project from [Bulebots](https://github.com/Bulebots/ommr) that can create maze files using images of them.

## Notes

- If you're using Windows, you may need to download and install [MinGW](http://mingw.org/wiki/Getting_Started)
- Communication with the simulator is done via stdin/stdout, use stderr to print output
- Descriptions of all available API methods can be found at [mackorone/mms#mouse-api](https://github.com/mackorone/mms#mouse-api)
