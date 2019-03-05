# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
<p align="center">
<a href="https://www.youtube.com/watch?v=KKQ2XqPRWwo"><img src="./path_planning.gif" alt="Path Planning" width="70%" height="70%">
</a>
<br> Path Planning
</p>
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
## Implementation

The implementation includes
* Addition of car_control.cpp
* Modification of main.cpp

Let's see these file in detail now.
### `car_control.cpp` Implementation
This file is responsible for handling `Vehicle` class two functions `check_safe_distance` and `predict_vehicle_future_s`.
1. `Vehicle` class
This class handles data coming from `sensor_fusion` and has variables named as `s`, `d`, `vx`, `vy`, `lane`, `speed`.
Depending on the `d` value of car and lane width which is 4 meters, its lane is calculated.
```cpp
  if (d < 0) {
    lane = -1;
  }
  else if ((0 <= d) & (d < 4))
  {
    lane = 0;
  }
  else if ((0 <= 4) & (d < 8))
  {
    lane = 1;
  }
  else if ((0 <= 8) & (d < 12))
  {
    lane = 2;
  }
```
Also, the `speed` is calculated using `vx` and `vy` values.
```cpp
speed = sqrt(vx * vx + vy * vy);
```

2. `check_safe_distance` function
This function accepts ego car's `s` value and detected vehicle's `s` value and returns
    * `true`: if ego car is behind detected vehicle AND distance between them is more than the `safe_distance`
    * `false`: otherwise

3. `predict_vehicle_future_s` function
This predicts the detected vehicle's `s` value in future. This is helpful, when the ego car is about to change the lane but there's a fast moving vehicle in the same lane where ego car will be shifting.
Using this function, the ego car will predict that, if it switches the lane to that particular lane where a fast moving car is approaching from back, it will abort lane switch. Instead, it will slow down its speed to avoid collition to the car in front of her.
```cpp
if (lane_change && left_lane_free) {
    lane -= 1;
    ref_vel -= 0.224 * 1.5;
}
```

### `main.cpp` Implementations
In this file, we've taken <b>4</b> states for our path planning's state machine.
1. `too_close`: this checks if the car detected via sensor_fusion is too close (< 30m close)
2. `left_lane_free`, `right_lane_free`: if above condition holds true, it will check whether the left or right lane is free and will set the value accordingly
3. `prepare_lane_change`: based on above both conditions true, will prepare to lane change
4. `lane_change`: when done preparing lane change, lane will be changed

For speed control, we're creating 3 waypoints 30m aparts from each other. Using `spline`, we're traversing through these points to create our total `50` waypoints for car to move forward.
```cpp
vector<double> next_wp0 = getXY(car_s + 30, (lane_width * lane + lane_width / 2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s + 60, (lane_width * lane + lane_width / 2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s + 90, (lane_width * lane + lane_width / 2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
```

## Valgrind
Here, I've taken care of memory leaks. Have validated the `path_planning` binary with [valgrind](http://valgrind.org/) tool and found that there's no memleaks possible with this code.
See attached valgrind logs named `valgrind_logs.txt`.

```bash
==13585== LEAK SUMMARY:
==13585==    definitely lost: 0 bytes in 0 blocks
==13585==    indirectly lost: 0 bytes in 0 blocks
==13585==      possibly lost: 0 bytes in 0 blocks
==13585==    still reachable: 734,462 bytes in 219 blocks
==13585==         suppressed: 0 bytes in 0 blocks
```