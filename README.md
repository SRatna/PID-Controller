# PID Controller for Self Driving Car
Self-Driving Car Engineer Nanodegree Program

---

I have developed a PID controller in c++ that successfully drives the vehicle around the track by using the cross track error (CTE) provided by the driving simulator.

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

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Reflection

I will here describe the role of each hyperparameters and how I tuned them.

### Proportional (P)
This factor tries to reduce the error proportionally but it has the tendency to overshoot and thus setting large value will make the vehicle go out of track and it also causes the oscillation of the vehicle unlesss it is balanced by other factors.

### Integral (I)
This factor accumulates all the previous error terms and act upon it according to the provided coefficient. If we give large coefficient, it will surely takes the vehicle out of track so we need to choose same value for it and it mostly helps if the system has some internal bias like bias in the steering angle.

### Derivate (D)
This factor plays great role in the prevention of overshoot and oscillation. It takes into account the rate of change of error and thus it will dampen the sudden change in the error term and that inturn prevents overshooting. So, we need to choose somewhat bigger value for it.

### Tuning
Initially I started with following coefficients:

| K(P) |  0.5 |
| K(I)  | 0.025  |
| K(D)  | 0.75  |

Due to small K(D), the vehicle suffered from ocillations and it quickly got out of the track. I then played around and tried increasing K(D) upto 3 and it prevented overshooting and the vehicle didn't leave the track. Decreasing K(P) to 2 also helped in preventing oscillations and thus I came to the following final coefficients:

| K(P) |  0.2 |
| K(I)  | 0.0025  |
| K(D)  | 3.5  |
