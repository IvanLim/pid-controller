# PID Controller Project

## The goals / steps of this project are the following:

- Implement a PID controller
- Tune the PID coefficients (using twiddle)
- Get the car to run around the track in the [Udacity simulator](https://github.com/udacity/self-driving-car-sim/releases)
- Summarize the results into this written report

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

---

## Reflections

**Effects of PID parameters**
* **P parameter**
  * The P parameter seemed to control the large, general corrections in vehicle movement. At the beginning, tuning this parameter gave the biggest gains.
* **I parameter**
  * After twiddle, the I parameter settled at 0. Any attempt to increase/decrease this value resulted in the car making a hard turn in one direction. This is probably due to the fact that the simulator track was a simple one with little opportunity for systemic bias.
* ** D parameter **
  * The D parameter tended to affect how quickly the vehicle would respond to change. The higher the value, the faster the vehicle turned to correct itself.
