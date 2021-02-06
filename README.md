[//]: # (Image References)

[simulator]: ./output_images/simulator.png "Simulator"
[simulation]: ./output_images/simulation.gif "Simulation"
[flowchart]: ./output_images/flowchart.png "Flowchart"

# **Kidnapped Vehicle** 

## Report

---

**Kidnapped Vehicle Project**
# Overview
This repository contains all the code needed to run the project for the Localization course in Udacity's Self-Driving Car Nanodegree.

## Project Introduction
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project a 2 dimensional particle filter is implemented in C++. The particle filter is given a map and some initial localization information (analogous to what a GPS would provide). At each time step the filter will also get observation and control data.

![alt text][flowchart]

## Prerequisites

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

Required tools are:
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
  
## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.
```sh
1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter
```
Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:
```sh
1. ./clean.sh
2. ./build.sh
3. ./run.sh
```
## Connecting with the simulator
Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Here is the main protocol that [`main.cpp`](https://github.com/mhusseinsh/CarND-Kidnapped-Vehicle-Project/blob/master/src/main.cpp) uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program
```
// sense noisy position data from the simulator

["sense_x"]

["sense_y"]

["sense_theta"]

// get the previous velocity and yaw rate to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values

["sense_observations_x"]

["sense_observations_y"]
```

OUTPUT: values provided by the c++ program to the simulator
```
// best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"]

//Optional message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label

["best_particle_associations"]

// for respective (x,y) sensed positions

["best_particle_sense_x"] <= list of sensed x positions

["best_particle_sense_y"] <= list of sensed y positions
```

To run the Particle Filter after building and installing: either `./particle_filter` or [`./run.sh`](https://github.com/mhusseinsh/CarND-Kidnapped-Vehicle-Project/blob/master/run.sh). The output will be as below:
```sh
Listening to port 4567
```
Here the Particle Filter will be waiting for the simulator to launch and start, and once it started, the below message will appear in the console.
```sh
Connected!!!
```
Initially, the simulator looks like this

![alt text][simulator]

# Implementing the Particle Filter
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

## Inputs to the Particle Filter
You can find the inputs to the particle filter in the [`data`](https://github.com/mhusseinsh/CarND-Kidnapped-Vehicle-Project/tree/master/data) directory.

#### The Map
[`map_data.txt`](https://github.com/mhusseinsh/CarND-Kidnapped-Vehicle-Project/blob/master/data/map_data.txt) includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
```
1. x position
2. y position
3. landmark id
```
### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.

## Results Evaluation and Success Criteria
Based on the defined [Rubric Points](https://review.udacity.com/#!/rubrics/747/view) the Particle Filter should achieve the two below points:

1. **Accuracy**: The particle filter should localize vehicle position and yaw to within the values specified in the parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`.

2. **Performance**: The particle filter should complete execution within the time of 100 seconds.

Finally, if the success criteria are met, the simulator should output
```
Success! Your particle filter passed!
```
The Particle Filter was implemented and the results are achieved which are meeting the defined success criteria:

| Parameter | Value |
|------|-----------|
| x  |  0.113   |
| y  |  0.107   |
| yaw  |  0.004   |
| time  |  48.88   |

And here is a run example:

![alt text][simulation]

