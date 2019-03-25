# Autonomous Vehicle Model Predictive Controller

This project implements Model Predictive Controller (MPC) for an autonomous vehicle
driving in a simulator.  MPC receives the path waypoints from the simulator (Path Planner in a real car).  
Based on these waypoints, MPC solves for actuators (acceleration and steering angle inputs), such that the 
vehicle's state (Cross-Track Error _CTE_, Yaw Error _EPSI_, Velocity Difference from Reference) is optimal.

First thing, the program converts waypoints into the car's frame of reference.  This makes the calculations 
much simpler down the road.  The algorithm is able to minimize _yaw error_ in the car's frame of
reference and it makes sense; it's declination from 0 deg.  In the map coordinate system, minimizing yaw error does not make 
much sense, especially if the car goes around a curve; car's yaw (relative to map's 0,0 coord.) must be very 
different from the slope of the waypoint curve (desired slope). 

After the waypoints are converted into the car's coordinate system, a cubic curve is fit through them.
This yields 4 coefficients that are used by MPC to calculate _CTE_ and _EPSI_ for the predicted path.
At every point of the path, MPC minimizes _CTE_ and _EPSI_ (differences in position and direction from the waypoint curve).
The curve coeffs. are passed to MPC along with the current vehicle State.  Car's State is a vector of 6 elements:
(_x_, _y_, _psi_, _v_, _cte_, _epsi_) where
```
x, y - car's position
ψ - car's yaw
v - car's velocity
cte - cross-track error or distance from waypoint curve
epsi (eψ) - yaw error or car's direction deviation from the waypoint curve's slope at x,y
```

In the simulator, without actuator latency, the state of the vehicles passed to MPC would be
(_0_, _0_, _0_, _v_, _cte_, _epsi_).  _x_, _y_, _psi_ are zero in the car's frame of reference.
To simulate a real vehicle with turning and acceleration actuation taking time, the code introduced
a 100 ms delay.  To account for the delay, the code sets the initial state to a predicted car location
100 ms down the road.  

MPC is using the initial State and the "bicycle" model of the car motion to predict a curve
of the future car travel.  The path is predicted  according to the following update equations:
```
x := x + v*cos(ψ)* dt
y := y + v sin(ψ) dt
v := v+a∗dt constrained to _a_ in [-1,1]
ψ := ψ - (v/L_f)*δ*dt constrained to δ in [-0.436, 0.436] radians (max possible car's turn range)
```

MPC class relies on Ipopt library to optimally predict the curve.  It uses a cost function that
"penalizes" high _cte_, _eψ_, sharp turns and acceleration changes.  The result of the MPC algo
is a locally optimal set of actuators (accelerations and turns) at every prediction point.
Only, the 1st actuators are feed back to the simulator; the rest are discarded.  The prediction 
process repeats during next cycles.

As part of the algorithm tuning, I had to choose the number of the car prediction points N.
In the lecture, it was suggested that the Receding Horizon should cover only a couple of seconds into the future.
I first tried N=25 and dt=0.05 per the class quiz.  This is roughly equivalent to 1.25 second horizon.
Albeit, the car crashed due to the prediction horizon reaching much further than the waypoint line.
Hence, I chose to reduce the horizon to N=10 and dt=0.1, which is equivalent to 1 second ahead.
The car successfully drove around the track.

To run the project and for more details refer to the original Udacity
[README](./README_orig.md).