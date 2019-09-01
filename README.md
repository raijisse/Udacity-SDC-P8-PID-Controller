# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

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


## Description of the PID Controller

A Porportional-Integral-Derivative (PID) controller is a kind of controller that uses feedback loops to keep the signal instruction at the desired level. In our use case, this means that we are going to use the deviation from the center line (Cross Track Error, CTE) as feedback to steer the vehicle back to the center of the track. The steering angle is therefore the value we are trying to set at the correct value and the target of our PID.

However, a PID controller must be carefully tuned as badly tuned controller may overshoot or undershoot, resulting in going out of the track because we turned too much or not enough.

As its names indicates, a PID controller has three main components that we are going to be able to tune : the P component, the I component and the D component.

- P stands for Proportional. In a P controller, we simply apply a factor (hence the proportionality) to the cross track error. Therefore, when the deviation from the center line is large (high CTE), the resulting steering angle will be large as well. If the P component is too big, we risk to perpetually overshoot.

- I stands for Integral. It applies a factor to the sum over all previous cross track errors. Therefore, it is proportional to the magnitude and the length of the error, as it is accumulated over time. However, when the error is negative, the I component will decrease while we still have negative error. Therefore, the I component may act with a little delay.

- D stands for Derivative. It uses the difference between the current CTE and the previous one and can be seen as the slope of the CTE. The bigger the slope, the bigger the correction brought by the steering angle will be. 


## Tuning Parameters
I used trial and error to tune the parameters, although twiddle can be used. I first increased the `Kp` parameter until the car started oscillating. Then increased `Kd` until the result look sufficient. I finally played a bit with `Ki` to try to reduce the bias.   


## Result
Overall, there was a pretty wide range of parameters that were able to drive the car around the track, some with a bit more of an oscillating trajectory, others a bit smoother. I chose final parameters `Kp` to be 0.125, `Ki` to be 0.0009 and `Kd` to be 1.8. These parameters gave me the results I was satisfied the most with. I noticed that the car tends to get close to the border of the lane but had smoother correction compared to other sets of parameters.

A first method to improve my results would be to use the twiddle method to find optimal parameters. However, there are other axis of improvement.
For instance, we could be done such as using another PID to control throttle and adapt throttle and speed depending on the turn angle. This could reduce speed in tight curves so that the PID controller suffer less of lagged decision and trying to overcorrect a bit late. 