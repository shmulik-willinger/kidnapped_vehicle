# Finding a Kidnapped Vehicle

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Overview
---

In this project the goal is to implement a **2 dimensional particle filter** in C++. The particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step the filter will also get observation and control data, and its goal is to find the kidnapped vehicle based on this data.

The communication between the project and the simulator is done using WebSocket.

This project involves the Udacity Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases). The simulator provides the script the noisy position data, vehicle controls, and noisy observations. the script feeds back the best particle state.

The WebSocket and other initial info regarding dependencies  installation can be found  [here](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project)

The Particle Filter Algorithm Flowchart:
![]( https://github.com/shmulik-willinger/kidnapped_vehicle/blob/master/readme_img/Particle_Flowchart.jpg?raw=true)

Prerequisites and Executing
---

This project requires the following dependencies:

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

**Build Instructions:**

The main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter
6. You will get the following output:
 `Listening to port 4567. Connected!!! `
7. Run the Simulator on `project 3` and start the process.

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

WebSocker communication
---

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

* sense noisy position data from the simulator :
["sense_x"]
["sense_y"]
["sense_theta"]

* get the previous velocity and yaw rate to predict the particle's transitioned state :
["previous_velocity"]
["previous_yawrate"]

* receive noisy observation data from the simulator, in a respective list of x/y values :
["sense_observations_x"]
["sense_observations_y"]

OUTPUT: values provided by the c++ program to the simulator

* best particle values used for calculating the error evaluation :
["best_particle_x"]
["best_particle_y"]
["best_particle_theta"]

* Optional message data used for debugging particle's sensing and associations

* for respective (x,y) sensed positions ID label :
["best_particle_associations"]

* for respective (x,y) sensed positions :
["best_particle_sense_x"] <= list of sensed x positions
["best_particle_sense_y"] <= list of sensed y positions


Implementing the Particle Filter
---

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

The file `src/main.cpp` contains the code that will actually be running the particle filter and calling the associated methods.

The file `particle_filter.cpp` contains the main implementation of this project and includes the following methods:
* init - Set the particles and Initialize them to first position with random Gaussian noise for x, y and theta (yaw)
* prediction - Add measurements and random Gaussian noise to each particle
* dataAssociation - Find the predicted measurement that is closest to each observed measurement and assign the observed measurement to this particular landmark
* updateWeights - Update the weights of each particle using a mult-variate Gaussian distribution.
* resample - Resample particles with replacement with probability proportional to their weight.

Process results
---
The particle filter met the requirements, and after 100 seconds the simulator gets out the following message: `Success! Your particle filter passed!`

![]( https://github.com/shmulik-willinger/kidnapped_vehicle/blob/master/readme_img/simulator_success.jpg?raw=true)

The green lines below are the car's observations of surrounding landmarks

The video below shows what it looks like when the simulator successfully is able to track the car to a particle:

[![video output](https://github.com/shmulik-willinger/kidnapped_vehicle/blob/master/readme_img/particle_simulator.gif)]
