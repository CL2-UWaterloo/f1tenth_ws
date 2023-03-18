# Wall Follow 
Personal notes From Lab3 of F1TENTH, original lab handout can be found [here](https://github.com/f1tenth/f1tenth_labs/blob/S2023/lab3/latex/lab3.pdf).

The wall follow node implements a simple PID controller for the car to autonomously drive forward while staying centered.

We have two control objectives:
1. Keep the car driving along the centerline ($y = 0$)
2. Keep the car parallel to the walls ($\theta = 0$)
	1. We define as after driving $L$ meters, we are still on the centerline. ($L\sin\theta = 0$)

Thus, we can define the error term as  
$$e(t) = -(y + L\sin\theta)$$



As a reminder, the PID Equation is given by
$$u(t) = K_{p}e(t) + K_{i} \int_{0}^{t}e(t')dt'+K_{d}\frac{d}{dt}(e(t))$$
where 
- $u(t)$ is the control output (in our case is the steering angle we want the car to drive at)
- $K_{p}$,$K_{i}$ and $K_{d}$ are constants that determine how much weight each of the three components (proportional, integral, derivative) contribute to the control output $u(t)$
- $e(t)$ is the error term. This is the difference between the set point and the parameter we want to maintain around that set point.


In practice, the PID calculations for the error term is calculated as
- $\frac{de(t)}{dt} \approx \frac{e_{t} - e_{t-1}}{\Delta t}$, i.e. take the differences the error between a timestep divided by the time elapsed for a timestep
- $\int{e(t)}{dt} \approx \sum\limits{e(t) }{\Delta t}$, i.e sum the errors and multiple it by the time elapsed since the beginning measurement

## Deriving the Error Term
In the context of our car, the desired distance to the wall should be our set point for our controller, which means our error is the difference between the desired and actual distance to the wall. 

To measure the distance to the wall,we consider the distance to the right wall $D_t$ at current time $t$.

Let's consider a generic orientation of the car with respect to the right wall and suppose the angle between the car's x-axis and the axis in the direction along the wall is denoted by ![](https://latex.codecogs.com/svg.latex?\alpha). 

We will obtain two laser scans (distances) to the wall: one 90 degrees to the right of the car's x-axis (beam b in the figure), and one (beam a) at an angle $\theta$ ( $0 < \theta \leq 70$  degrees) to the first beam. Suppose these two laser scans return distances a and b, respectively.

![fig1](img/wall_following_lab_figure_1.png)

*Figure 1: Distance and orientation of the car relative to the wall*

We can express $\alpha$ as 
$$\alpha = \tan^{-1}\left( \frac{a\cos(\theta)-b}{a\sin(\theta)} \right)$$


We can then express $D_t$ as  $D_t = b\cos(\alpha)$ to get the current distance between the car and the right wall. 

Our error term $e(t)$ is simply the difference between the desired distance and actual distance, for example $D_{target} = 1$, so 
$$e(t)= D_{target} - D_t$$

	
Since we are operating at high speeds, we must look to the future and project the car ahead by a certain lookahead distance (let's call it $L$). Our new distance $D_{t+1}$ will then be
$$D_{t+1} = D_t + L\sin(\alpha)$$


![fig1](img/wall_following_lab_figure_2.png)

*Figure 2: Finding the future distance from the car to the wall*

We will also add Speed control for when we are driving straight vs. turning:
- If the steering angle is between 0 degrees and 10 degrees, the car should drive at 1.5 meters per second.
- If the steering angle is between 10 degrees and 20 degrees, the speed should be 1.0 meters per second.
- Otherwise, the speed should be 0.5 meters per second.

## Summary of steps
1. Obtain two laser scans (distances) $a$ and $b$.
2. Use the distances $a$ and $b$ to calculate the angle $\alpha$.
3. Use $\alpha$ to find the current distance $D_t$ to the car, and then $\alpha$ and $D_t$ to find the estimated future distance $D_{t+1}$ to the wall.
4. Run $D_{t+1}$ through the PID algorithm described above to get a steering angle.
5. Use the steering angle you computed in the previous step to compute a safe driving speed.
6. Publish the steering angle and driving speed to the `/drive` topic in simulation.
