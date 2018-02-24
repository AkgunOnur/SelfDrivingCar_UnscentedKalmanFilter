# Unscented Kalman Filter

[//]: # (Image References)

[image1]: ./examples/dataset1.jpg "Results for Dataset 1"
[image2]: ./examples/dataset2.jpg "Results for Dataset 1"

## Overview

Due to the fact that the Extended Kalman Filter may give poor results in case of nonlinear motion, Unscented Kalman Filter is implemented in C++ to estimate position and velocity of an object by using Radar and Lidar measurements in this project. For a self driving car to drive safely, it needs to detect the positions and the velocities of the objects around. In order to accomplish this task, multiple sensor measurements (Laser and Radar) are combined by a Sensor Fusion technique which is Unscented Kalman Filter. To visualize the process, the simulator that Udacity provides is used. The communication between the simulator and the UKF is done by using uWebSockets implementation on the UKF side. 

## Prerequisites

The project has the following dependencies :

* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4
* Udacity's simulator (can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases) )

In order to install necessary libraries on Windows, [install-ubuntu.sh](./install-ubuntu.sh) needs to be executed.

## Compilation and Execution the Project

* Clone the repo and cd to it on a Terminal.
* Create a build file and cd to it `mkdir build && cd build`
* Compile it `cmake .. && make`
* Execute it `./UnscentedKF`

## Results

As it can been seen below, Unscented Kalman Filter gives better tracking results than Extended Kalman Filter (can be compared with [this] (https://github.com/MeRKeZ/SelfDrivingCar_ExtendedKalmanFilter))

### Results for Dataset 1

![alt text][image1]

### Results for Dataset 2

![alt text][image2]
