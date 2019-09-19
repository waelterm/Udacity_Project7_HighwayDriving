# Udacity: Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
[//]: # (Image References)

[image1]: ./images/capture1.png "Grayscaling"


### Simulator.
This repository include my solution to the Highway Driving Project of the Udacity Self-Driving Car Nanodegree.

### Decision Making
One of the primary tasks of this project was to create an algorithm to change lanes.
The decision algorithm can be found in line 128 - 404 of the main.cpp file.

We are analyzing the situation depending on what lane the vehicle is currently in.
For each of the three states (Driving in left, center, or right lane), five actions are considered:
* Stay in Current Lane
* Left Lane Change
* Double Left Lange change
* Right Lane Change
* Double Right Lane Change

The first option will be taken if the egovehicle has no slower vehicle in front of it or if no save option with a 
utility value of 10 is available.

For the other four actions, a utility value and a flag deciding whether the action is save are determined.
The action is safe, if there is a large enough gap, and no vehicle from behind is driving so fast that it will 
enter this gap during the lane change time of three seconds. Additionally, it is checked that there exists a target 
lane. You cannot make a left lane change if you are in the lef lane.
For the double lane changes, this has to be the case for both lanes, and the second lane has an even bigger
gap and time requirement.

For each action a utility value is calculated. There are three different ways these utility values are calculated
1. If the lead vehicle in that target lane is slower than the current lane speed, the time until that vehicle will be 
reached at current speed is calculated.
2. If the lead vehicle in the target lane is faster than the ego, the speed difference multiplied by a tunable gain factor is calculated.
3. If the lane is empty, the utility i set to a high constant value.

After these calculations, a save action with the highest utility value will be chosen. Unless the utility value is lower 
than the threshold of 10 or the vehicle is already achieving its target speed in the current lane.


### Path Generation
The code for the longitudinal path generation can be found in lines  130 -155 and 408 - 500.

For the reference speed calculations, a desired speed is calculated using the distance and velocity difference to the lead vehicle.
```
prel_desired_vel = check_speed * 2.24 - (check_speed * 2.24 / 20) * (20 - delta_s)
```

If there is no lead vehicle, the desired speed will be 49.5 mph.
Based on the desired target, a reference acceleration will be calculated and applied to get the reference velocity of future points.
The reference acceleration will get additional limitation during lane changes. There is also a jerk limiter implemented.

The reference velocity can then be used to calculate the distance between the waypoints.
To get the waypoints, I fit a spline through the last waypoint of the previous iteration, and three points on the target lane with 30m increments.

These points were first found in Frenet coordinates and then transformed into cartesian coordinates.
The spline is then used to find five waypoints. The distance of those points depends on the reference velocity at each waypoint.
I decided to use only five waypoints after realizing that the code runs so fast that only a single point is used by the simulator at each iteration.
Therefore, I decided to use only five waypoints in order to be able to quickly react to the most recent perception data.
The five waypoints are then being pushed to the simulator.

If a lane change is initiated, the same procedure will be used to generate the path. However, the reference lane used 
to find reference points for the spline will be different.

### Performance
I was able to have the car running for 30 minutes and it completed a total of 22.82 miles, averaging a speed of 45.64 mph.

![alt text][image1]

### Future Improvements
1. No active gap search: Improve vehicle to speed up or slow down in order to find gap in faster moving lane.
2. Give bias to center lane: Center lane is preferred because it gives two options for lane changes. This makes it easier to avoid being stuck behind a slower vehicle.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

