# Path Generation Model
### Introduction
The path generation model has two main components: behavior planning and trajectory generation. For behavior planning, a finite state machine is used to keep track of the cars state. The transition function of the state machine uses weighted cost functions to determine the optimal state. The behavior of the main function is determined by the state of the FSM at any given time. The main function checks the desired speed and lane, which is based on the state of the FSM, then it generates cubic polynomial spline trajectories to achieve the desired result with smooth motion. 

### Finite State Machine
The finite state machine used for this program has 5 states:
* Ready
* Lane Keep
* Prepare Right Turn
* Prepare Left Turn
* Turn Right
* Turn Left

The model in Figure 1 shows the possible transitions between the states. 

![alt text](/images/finite_state_machine.png "Finite State Machine")

Figure 1: Finite State Machine 

### Spline Trajectory Generator 

##### Overview
The path planning program employs a spline library (tk spline) to generate smooth and drivable cubic polynomial spline trajectories. Splines were chosen for trajectory generation because the spline library was readily accessible, easy to use, and met the low-jerk requirement, however, other functions such as quintic polynomials could be used. 

##### How the Spline Trajectories are Generated
First, the 2 most recent path points remaining from the last cycle are added to the spline. If no previous path points are available, which would be the case at initialization, the car's current position and angle are used to extrapolate the previous points. Starting with the previous points ensures that there is no continuity break between cycles, which would result in breaking the jerk limit. 

Next, 3 map waypoints spaced roughly 30, 60, and 90 meters ahead in the appropriate lane are added to the spline. Frenet coordinates are used to specify the waypoints for convenience, but positions are converted to x, y coordinates before being added to the spline. 

The spline is based on 5 total points the two previous path points and the 3 waypoints.  

Finally, the spline is sampled up to 30 meters ahead of the car. The number of points sampled depends on the desired speed. The space between points determines how much the car travels in one cycle, which determines its effective speed. Therefore, the more points sampled over a given distance the slower speed. After each point is sampled from the spline it is converted into global (x,y)  coordinates, to meet the requirements of the simulator. Then, all of the sampled points are passed into the simulator and the simulator moves along trajectory for one cycle. 
