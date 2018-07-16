# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Introduction

The purpose of the project is to impelement PID-controller, that controlls car on the track and to tune its parameters 
using simulator data.

---

## PID controller parameters

Given a cross-track error (CTE) PID controller produces steering value that helps keep car on the track. PID controller
consists of three components `P` (proportional), `I` (integral) and `D` (differential).

`P` component is the main component of PID contoller, it has the most influense on car behaviour. It corresponds to the part
of steering value that is proportional to cross-track error. So if car dodges to the right side of the center of the lane it
helps car to steer left (opposite direction) and vise versa. `P` component is proportional to cross-track error so its part in steering value is
`Kp * CTE`, where CTE is cross track error and `Kp` is a coefficient. Note that as we apply steering value with opposite sign,
`Kp > 0` despite steering is applied in opposite direction. The same applies for `D` and `I` components. Though arguably `P` 
component helps the most to hold car on the trajectory it is prone to overshooting and sine-like divergence of the trajectory of
movement so other components should be applied as well.

`D` component is proportional to the rate of change of CTE so its part in steering value is `Kd * dCTE`, where `dCTE = CTE(n) - CTE(n-1)`
and `Kd` is a coefficient. `D` component is used to improve trajectory of movement acquisition. Its purpose is to mitigate overshooting
(and as a result sine-like divergence) of `P`-controller.

`I` component is proportional to the sum of CTEs so its part in steering value is `Ki * sum(CTE)`, where `sum(CTE)` is the sum of all cross track
errors up to now and `Ki` is a coefficient. `I` component is used to mitigate bias in cross track error values so the car can move along center of the lane.

---

## Parameters tunning

I decided to use manual tunning process as to use twiddle tunning a car should already be able to move along track in a correct manner. In my opinion
at this point it would be hard to obtain a significant improvement in car behaviour. I took approach that was suggested on Slack channel to implement
a `P`-controller at first that move to `PD`-controller than to `PID`-controller.

My first step was to implement a `P`-controller. I needed to find a balance between car's ability to take turns and trajectory divergence. I started with
`Kp = 1` and got a car movement that diverged in a very fast manner. Then I started to decrease `Kp` and ended up with `Kp = 0.05`. With that parameter car 
was able to pass first two turns than its trajectory diverged. At that moment I decided to move to the next step. Resulting video of the first step is
at [P_controller](https://youtu.be/gD2REJu88ds) (`Kp = 0.05, Kd = 0., Ki = 0.`).

Second step was to add `D` component and implement `PD`-controller. At first I tried to find a `Kd` that was smaller than `Kp`. Adding small `Kd` did
not help me. Than I realised that `D` component worked with rate of change of CTE that was much smaller than CTE itself. So I needed to have `Kd` bigger
that `Kp`. I started with `Kd = 1` and got a significant improvement. Than I realised that `Kp` was not big enough as car clearly had problems in sharp turns.
I increased `Kp` to `0.1` and got a quite nice trajectory of movement (though it had some troubles in sharp turns) as `D` parameter really helped me to mitigate 
divergence. I started to further increse `Kd` and ended up with `Kd = 8.` (I tried bigger `Kd`s but it resulted in worse behaviour, trajectory of movement became 
`nervous`/had spikes). At this moment car already met specifiacation (did not touch boundaries of lane of movement) so I decided to move to final step. Resulting 
video of the second step is at [PD_controller](https://youtu.be/_Ve2QOPF0Gs) (`Kp = 0.1, Kd = 8., Ki = 0.`).

Third step was to add `I` component and implement `PID`-controller. As `I` component deals with sum of cross track errors I decided to search for a much smaller
`Ki` (relative to `Kp` and `Kd`). I started with `Ki = 0.0005` and it gave me an improvement than I encreased `Ki` to `0.001` to have an even bigger improvent. At that
moment I obtained a controller that seemed good enough for me so I decided to stop at that moment. I added the last tweak to decrease `Kd` to `6.` to have a little 
smoother car movement. Resulting video of the final step is at [PID_controller](https://youtu.be/snor3RK8peQ) (`Kp = 0.1, Kd = 6., Ki = 0.001`).

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

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

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

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
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

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

