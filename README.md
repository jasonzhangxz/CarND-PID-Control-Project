# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
## OVerview
This project is to build a PID controller and tune the PID hyperparameters and make the vehicle drive successfully around the track without going out of the track.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.


## Implementation
The PID controller is implemented in [./src/PID.cpp](./src/PID.cpp). The [PID::UpdateError](./src/PID.cpp#L27) method calculates proportional, integral and derivative errors and the [PID::TotalError](./src/PID.cpp#L37) calculates the total error given the coefficients.

## Reflection
- The Proportional part of the controller is trying to correct the Cross Track Error (CTE). If used alone, it will overshoot and oscillate, so the car may bounce between the left and right lane boundary and can easily get off the road, as shown in this video [./videos/P_only.mp4](./videos/P_only.mp4).
- The Integral part of the controller is trying to correct the bias of the control system, which could be a steering angle error. But there doesn't seem to be a bias on the simulator, so when Integral used alone here, it actually over correct it, so as shown in the video it is doing a circle [./videos/I_only.mp4](./videos/I_only.mp4).
- The Derivative portion of the controller is trying to reduce the overshoot of the proportional gain. So most of time P-D work together in a control system. But used D alone doesn't do a lot. This video [./videos/D_only.mp4](./videos/D_only.mp4) shows the simulation of a D only control.

I chose the final parameters by trial and error. As the simulator doesn't seem to introduce a system bias, I set the Integral portion to be 0.0, Then add a proportional gain to correct the CTE error, as it overshoots and go out of bounds, start to add Derivative control to smooth the overshooting.

The final parameters I chose are [P:0.2 ,I:0 ,D:2.5], a video of the simulation run is here [./videos/finalParam.mp4](./videos/finalParam.mp4). 
