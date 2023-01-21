# Wall Follow 
The wall follow node implements a simple PID controller for the car to autonomously drive forward while staying conetered.

We have two control objectives:
1. Keep the car driving along the centerline ($y = 0$)
2. Keep the car parallel to the walls ($\theta = 0$)
	1. We define as after driving $L$ meters, we are still on the centerline. ($L\sin\theta = 0$)

As a reminder, the PID Equation is given by
$$u(t) = K_{p}e(t) + K_{i} \int_{0}^{t}e(t')dt'+K_{d}\frac{d}{dt}(e(t))$$
where 
- $u(t)$ is the control output (in our case is the steering angle we want the car to drive at)
- $K_{p}$,$K_{i}$ and $K_{d}$ are constants that determine how much weight each of the three components (proportional, integral, derivative) contribute to the
- $e(t)$ is the error term. This is the difference between the set point and the parameter we want to maintain around that set point.


In practice, we have that
- $e(t)$ is the measured error at time $t$, which we define as $e(t) = -(y + L\sin\theta)$
- $\frac{de(t)}{dt} \approx \frac{e_{t} - e_{t-1}}{\Delta t}$, i.e. take the differences the error between a timestep divided by the time elapsed for a timestep
- $\int{e(t)}{dt} \approx \sum\limits{e(t) }{\Delta t}$, i.e sum the errors and multiple it by the time elapsed since the beginning measurement
