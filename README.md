# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
## Goal of the project
The goal of the project is to implement a PIC controller in C++. This controller should control the steering angle in a manner to execute a drive around a simulator track.

## PID Control
Sensor measurements regarding the car's position is compared to the desired position. The deviation is ascertained as the cross-track-error (cte). The CTE is thereby fed into a PID controller loop to estimate
the necessary control output (steering wheel angle) that is required to minimize any existing error.

Proportional component P: A proportional component would try to correct the CTE by applying a control signal of Kp * cte.

Derivate component D: A derivative component which controlling the cte's rate of change can be integrated to counteract the overshooting. This is done by adding a term of Ki * (cte - prev_cte), where prev_cte is the error of the previous timestep.

Integral component I: An integral component of Ki * sum(cte) is added to the control output in order to eliminate the steady state bias (where sum(cte) is the sum of all previous errors).

## Implementation in Code
The implementation of the PID controller is done in the PID.cpp and PID.h file.

In the PID::UpdateError() function the P, I and D error terms are computed based on the (CTE). Then in the PID::TotalError() function the control output is calculated as the sum of the individual control terms.

double output = -Kp * p_error - Kd * d_error - Ki * i_error;

The control signal for steering is limited to a range between -1 and 1 as instructed.

After initiation of the controller, the output is passed to the msgJson["steering_angle] which is connected to the simulator.

## Adjusting the parameters of the PID
The fine tuning of the Kp was initially done to first see its sensitivity. This was followed by the derivate factor so as to enable the vehicle drive around the track. It was seen that the vehicle was not
always following the center. Hence the integral part was parameterized to enable the controller to follow a nice path.

Many instructions online were used to find tips on manually adjusting these parameters. The final parameters are seen in the code.

The controller for speed is tuned according to the steering angle error. The lower the error, the higher is the target speed. Likewise, if the error increases, the speed of the vehicle is reduced.

## Remarks and Observations
Many a times, the simulation did not run on my PC, but was executable on another computer. Furthermore, running on the UDACITY Workspace showed that the controller was more twichy than on the PC. It is clear
that the simulation delay has a big sensitivity to the PID parameters. Hence, it is uncertain whether the PID Controller would work harmoniously on a different setup.

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

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

