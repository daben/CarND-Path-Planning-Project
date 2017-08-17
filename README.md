# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

---

![[Video][video.mp4]](video.gif)
*Download [video.mp4](video.mp4?raw=true) for the full video.*

## Description

The goal of this project is to design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic. See below for a more detailed description of project.

The solution consists of the following files: 

* `main.cpp`: interfaces with the simulator and invokes the path planner.
* `planner.hpp`: implements the path planner. Given the telemetry data provided from the simulator generates a new path for the car in world coordinates.
* `spline.h`: cubic spline interpolation library by Tino Kluge used by the trajectory generator.
* `json.hpp`: JSON library for C++ used to interface the simulator.

*Note that Eigen is not used in this solution.*

The localization component used by the planner is built over the waypoint list provided. See the classes `RoadMap` and `Track` in the file `planner.hpp`.

The planner itself proceeds in six steps: 

#### 1. Compute the reference point

It sets two points from the end of the previous path, or the current position of the car if the previous path has been consumed, that allow to create a tangent line in the direction of the car.

See method `PathPlanner::compute_reference`.
    
#### 2. Process the sensor fusion data 

The data from the sensor fusion is analysed and a table with the state of all the lanes is produced. For each lane, a struct `lane_info_t` is filled containing information about the cars ahead and behind of the reference position: their unique id, their gap and their speed. 
    
This is the definition of the lane info struct with is default values:
    
```C++
struct lane_info_t
{
    int    front_car   = -1;
    int    back_car    = -1;
    double front_gap   = INF;
    double front_speed = INF;
    double back_gap    = INF;
    double back_speed  = 0;
    bool   feasible    = true;
};
```
A **feasibility** flag signaling if the gap in the lane is large enough for a lane change is computed here for simplicity in the following manner:
    
```C+
lane.feasible =    (lane.front_gap > lane_change_front_buffer)
                && (lane.back_gap > lane_change_back_buffer);
```

See method `PathPlanner::process_sensor_fusion`.
    

#### 3. Decide next plan

The behavior planner is designed as a simple state machine with three states: *keep lane* (KL), *prepare for lane change* (PLC) and *lane change* (LC). The telemetry and the lane information trigger the transitions between states. KL is the initial state.

The planner is able to plan for a change to a not adjacent lane, however the plan is realised first changing to the lane in the middle. 

The output of the planner are the target lane (variable `target_lane`) and the desired target speed (`target_speed`) that the next stages should consider.

See method `PathPlanner::create_plan`.

##### Keep Lane State

On enter fix the target speed to road limit.

If there is car in front forcing a speed below the road limit and there is a faster lane, set that lane as change target (variable `changing_lane`) and transition to PLC.

The faster lane is decided in the method `PathPlanner::get_best_lane`.

##### Prepare for Lane Change State

On enter, fix the target lane as the changing lane, of the closest lane in the direction of the changing lane. 

If the target lane is feasible, transition to lane change.

If not feasible and the changing lane is not the fastest or we have been in this state for too long, abort and transition back to keep lane.

Otherwise, wait for an opportunity to change the lane, adjusting the speed if necessary.

##### Lane Change State

When current lane change completed, if it's not the final lane, transition to PLC, else to KL.

If a risk of collision is detected in the middle of a lane change the change is aborted fixing the lane target to the reference.

#### 4. Collision avoidance

A simple mechanism to avoid collisions with cars in the target lane reducing the speed. The output of this component is a maximum safe target speed.

See method `PathPlanner::collision_avoidance`.

#### 5. Speed control

A simple mechanism to accelerate in a safe manner. If defines de final target speed for the trajectory generator.

#### 6. Trajectory generation

Generates a smooth trajectory from the reference point to the target lane and to the target speed. The trajectory is generated using a cubic spline interpolation in the manner described in the walkthrough video.

---

## Project instructions

### Simulator
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

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

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
