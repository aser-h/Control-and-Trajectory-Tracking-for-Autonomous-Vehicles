# Control and Trajectory Tracking for Autonomous Vehicles
## Project Overview
### Project Summary
In this project,there will be application to designing a PID controller to perform vehicle trajectory tracking. A trajectory will be given as an array of locations, and a simulation environment (the vehicle with possible perturbations) will be provided. A PID controller will be designed and coded, and its efficiency will be tested on the CARLA simulator used in the industry.

The project will provide the dynamic model of the vehicle, as well as an approximate distribution of the possible perturbations. The code to use the simulation environment (CARLA) will also be provided, along with a template in C++ for the controller.

The deliverable will consist of the code, explanations about how the parameters were chosen, as well as a result of the simulation with analysis of the results.

### Project Steps Overview
* Design the PID controller in C++
* Integrate the controller with the CARLA simulator.
* Tune the parameters using a technique
* Create plots to show how successful the controller is as well as the simulator video.
### Project Dependencies Overview
* CARLA simulator in a workspace.
* A C++ solver open sources and used in the industry
* Code to interact with the CARLA simulator
___
## Installation
Installation can be done by running the following commands to install the starter code in the Udacity Workspace:

Clone the repository:

* git clone https://github.com/udacity/nd013-c6-control-starter.git

### Run Carla Simulator
Open new window

    su - student 
    cd /opt/carla-simulator/
    SDL_VIDEODRIVER=offscreen ./CarlaUE4.sh -opengl
### Compile and Run the Controller
Open new window

    cd nd013-c6-control-starter/project
    ./install-ubuntu.sh
    cd pid_controller/
    rm -rf rpclib
    git clone https://github.com/rpclib/rpclib.git
    cmake .
    make (This last command compiles your c++ code, run it after every change in your code)
### Testing
To test your installation run the following commands.

    cd nd013-c6-control-starter/project
    ./run_main_pid.sh This will silently fail ctrl + C to stop
    ./run_main_pid.sh (again) Go to desktop mode to see CARLA
    If error bind is already in use, or address already being used

    ps -aux | grep carla
    kill id

___

## Project Instructions
A steer and throttle controller will be built to make the car follow the trajectory.

A PID controller will be designed and run.

The files ```pid_controller.cpp``` and ```pid_controller.h``` can be found in the directory ```/pid_controller```. These files are where the PID controller will be coded. The function pid is called in ```main.cpp```.

### Step 1: Build the PID controller object

Run the simulator and see in the desktop mode the car in the CARLA simulator. Take a screenshot and add it to your report. The car should not move in the simulation.

INSERTING STATIONARY CAR HERE

### Step 2: PID controller for throttle:
In ```main.cpp```, there is a TODO (step 2) that needs to be completed in order to compute the error for the throttle PID. Specifically, the error is the speed difference between the actual speed and the desired speed.

___

## Results
### Car as shown in Carla simulator (not moving because main.cpp not completed yet)
![Alt text](media/Pic%20of%20car%20not%20moving.png)

### Here it is after main.cpp completeion (vid below)

![Alt text](media/Car%20moving%20till%20end%20of%20road.gif)


### Plots

![Alt text](media/Screenshot%20from%202023-05-06%2003-42-57.png)

![Alt text](media/Screenshot%20from%202023-05-06%2003-43-17.png)

## Question time..


### How would you design a way to automatically tune the PID parameters?

There are several ways to automatically tune the PID parameters. Here are some methods:

* 1- Ziegler-Nichols method.

* 2- Cohen-Coon method.

* 3- Relay feedback method.

* 4- Genetic algorithms.

* 5- Particle swarm optimization.

* 6- Model-based optimization.

With that being said the two easiest methods for automatic tuning of PID parameters are the Ziegler-Nichols method and the Cohen-Coon method.

* The Ziegler-Nichols method involves performing a step response test to determine the process parameters such as the critical gain and critical period, which are then used to calculate the PID parameters. This method is relatively easy to implement and requires minimal mathematical knowledge.

* The Cohen-Coon method also involves performing a step response test to determine the process parameters, but instead of using critical gain and critical period, it uses the ultimate gain and ultimate period to calculate the PID parameters. This method is slightly more complex than the Ziegler-Nichols method, but it is still considered relatively easy to implement.

And finally lets not forget aboyt the most important one which is Twindle, though I did not include it in the above list for a reason. That being said it is a variant of the reinforcement learning algorithm that can be used for PID controller tuning. It is based on the idea of balancing exploration and exploitation to find the optimal PID parameters.

In the Twindle algorithm, the controller's performance is evaluated using a reward signal that is based on the error between the desired setpoint and the actual system output. The reward signal is used to update the controller's parameters using a gradient-based optimization algorithm.

One advantage of the Twindle algorithm is that it can be used in a variety of systems, even those with complex dynamics, without requiring detailed knowledge of the system's underlying model. However, the Twindle algorithm may require a large number of iterations to converge on an optimal set of PID parameters, which can make it computationally expensive and time-consuming.





### PID controller is a model free controller, i.e. it does not use a model of the car. Could you explain the pros and cons of this type of controller?

One advantage of the PID controller is that it is a straightforward concept that can be used for a wide range of problems without alterations. Implementing a basic version of the controller for a new issue is easy because no expert knowledge is needed about the underlying system, and the parameters to be adjusted are general and simple to comprehend. Additionally, computing the output is computationally undemanding, making it suitable for real-time applications, even with limited resources, such as small microcontrollers.

However, it should not be assumed that achieving good control is effortless. Having a limited number of parameters means that discovering the ideal combination may not always be possible. Moreover, without an internal model of the system, it is impossible to incorporate important aspects of the system into the controller, requiring external consideration. For example, if we know that the effect of a control signal on the system depends on the current state, such as the current gear, we need to consider this outside of the controller by applying a different gain parameter to the output. A controller with an internal model could better handle factors like this, assuming the model is accurate, which necessitates expert knowledge or tedious identification measurements. Finally, delays are a significant challenge for a PID controller, which may be better addressed by controllers that conduct forward simulation.

### (Optional) What would you do to improve the PID controller?
* Tuning: Proper tuning of the PID parameters is critical to achieving good performance. Manual tuning can be time-consuming and may not always result in optimal performance. One way to improve the tuning process is to use automated tuning methods, such as the Ziegler-Nichols method, which can help determine the appropriate values for the proportional, integral, and derivative gains.

* Anti-windup: One limitation of PID controllers is that they can be prone to "windup" when the system is saturated, meaning that the integrator term continues to accumulate error even though the controller output is limited. Anti-windup techniques, such as back-calculation or dynamic control, can be used to prevent integrator windup and improve the stability of the system.

* Nonlinear control: In some cases, the process being controlled may have nonlinear dynamics that cannot be accurately represented by a linear PID controller. Nonlinear control techniques, such as fuzzy logic or adaptive control, can be used to improve the performance of the controller in these situations.

* Model-based control: PID controllers are often used in conjunction with mathematical models of the system being controlled. Model-based control techniques, such as model predictive control or adaptive control, can be used to improve the performance of the controller by taking into account the dynamics of the system and adjusting the control signal accordingly.

* Advanced algorithms: More advanced algorithms, such as fractional order PID, cascade control, or feedforward control, can be used to improve the performance of the controller in specific situations where traditional PID control may not be effective.

___
Update 2
## More Attempts
After hours of of adjusting and tweaking with the tuning parameters in the ```main.cpp``` specifically the ```pid_steer.Init``` and ```pid_throttle.Init``` I came to conclusion that not the same input will always give out the same output . In other words , running the code without changing anything (same tunings) will result in different outputs in the Carla simulation.
While my tuning parameters might give certain output, it might not do the same when someone else (or even me) repeat it.
So following a considerable amount of effort dedicated to refining and perfecting the almost stable tunings are `pid_steer.Init(0.3, 0.0008, 0.4, 1.2, -1.2);` and `pid_throttle.Init(0.2, 0.00087, 0.12, 1.0, -1.0);` Line '252' and '259' respectively in `main.cpp`
## Below here are results for the same above parameters
### Videos

![Alt text](media/Other%20attempts/2nd%20attmpt.gif)


![Alt text](media/Other%20attempts/3%20att.gif)


![Alt text](media/Other%20attempts/H%20attmpt.gif)

### More plots
1
![Alt text](media/Other%20attempts/2nd%20it.png)

![Alt text](media/Other%20attempts/2nd%20pid.png)

2

![Alt text](media/Other%20attempts/Plot1.png)

![Alt text](media/Other%20attempts/Plot2.png)

3

![Alt text](media/Other%20attempts/Plot3.png)

![Alt text](media/Other%20attempts/Plot4.png)


