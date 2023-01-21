# Lab 2: Automatic Emergency Braking

## I. Learning Goals

- Using the `LaserScan` message in ROS 2
- Instantaneous Time to Collision (iTTC)
- Safety critical systems

## II. Overview

The goal of this lab is to develop a safety node for the race cars that will stop the car from collision when travelling at higher velocities. We will implement Instantaneous Time to Collision (iTTC) using the `LaserScan` message in the simulator.

For different commonly used ROS 2 messages, they are kept mostly the same as in ROS 1. You can use `ros2 interface show <msg_name>` to see the definition of messages. Note for messages that are not installed by default by the distro we use in our container, you'll have to first install it for this to work.

#### The `LaserScan` Message

[LaserScan](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html) message contains several fields that will be useful to us. You can see detailed descriptions of what each field contains in the API. The one we'll be using the most is the `ranges` field. This is an array that contains all range measurements from the LiDAR radially ordered. You'll need to subscribe to the `/scan` topic and calculate iTTC with the LaserScan messages.

#### The `Odometry` Message

Both the simulator node and the car itself publish [Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) messages. Within its several fields, the message includes the cars position, orientation, and velocity. You'll need to explore this message type in this lab.

#### The `AckermannDriveStamped` Message

You've already used [AckermannDriveStamped](http://docs.ros.org/en/jade/api/ackermann_msgs/html/msg/AckermannDriveStamped.html) in the previous lab. It will be the message type that we'll use throughout the course to send driving commands to the simulator and the car. In the simulator, you can stop the car by sending an `AckermannDriveStamped` message with the `speed` field set to 0.0.

## III. The TTC Calculation

Time to Collision (TTC) is the time it would take for the car to collide with an obstacle if it maintained its current heading and velocity. We approximate the time to collision using Instantaneous Time to Collision (iTTC), which is the ratio of instantaneous range to range rate calculated from current range measurements and velocity measurements of the vehicle.

As discussed in the lecture, we can calculate the iTTC as:

<!-- , and the operator <img src="https://render.githubusercontent.com/render/math?math=[]_{%2B}" height="22"> is defined as: <img src="https://render.githubusercontent.com/render/math?math=[x]_{%2B}=\text{max}(x,%200)" height="22"> -->

<img src="https://render.githubusercontent.com/render/math?math=iTTC=\frac{r}{[- \dot{r}]_{%2B}}" height="50">

where <img src="https://render.githubusercontent.com/render/math?math=r" height="10"> is the instantaneous range measurements, and <img src="https://render.githubusercontent.com/render/math?math=\dot{r}" height="18"> is the current range rates, And the operator <img src="https://render.githubusercontent.com/render/math?math=[ ]_{%2B}" height="18"> is defined as <img src="https://render.githubusercontent.com/render/math?math=[ x ]_{%2B} = \text{max}( x, 0 )" height="18">. The instantaneous range <img src="https://render.githubusercontent.com/render/math?math=r" height="10"> to an obstacle is easily obtained by using the current measurements from the `LaserScan` message. The range rate <img src="https://render.githubusercontent.com/render/math?math=\dot{r}" height="18"> is the expected rate of change along each scan beam. Thus, it can be calculated by mapping the vehicle's current longitudinal velocity onto each scan beam's angle by using <img src="https://render.githubusercontent.com/render/math?math=v_x \cos{\theta_{i}}" height="18">. The angles could also be determined by the information in `LaserScan` messages. The range rate could also be interpreted as how much the range measurement will change if the vehicle keeps the current velocity and the obstacle remains stationary. Note the negation in the calculation this is to correctly interpret whether the range measurement should be decreasing or increasing. For a vehicle travelling forward towards an obstacle, the corresponding range rate should be negative since the range measurement should be shrinking. Vice versa, the range rate corresponding to the vehicle travelling away from an obstacle should be positive since the range measurement should be increasing. The operator is in place so the iTTC calculation will be meaningful. When the range rate is positive, the operator will make sure iTTC for that angle goes to infinity.

After your calculations, you should end up with an array of iTTCs that correspond to each angle. When a time to collision drops below a certain threshold, it means a collision is imminent.

## IV. Automatic Emergency Braking with iTTC

For this lab, you will make a Safety Node that should halt the car before it collides with obstacles. To do this, you will make a ROS 2 node that subscribes to the `LaserScan` and `Odometry` messages. It should analyze the `LaserScan` data and, if necessary, publish an `AckermannDriveStamped` with the `speed` field set to 0.0 m/s to brake. After you've calculated the array of iTTCs, you should decide how to proceed with this information. You'll have to decide how to threshold, and how to best remove false positives (braking when collision isn't imminent). Don't forget to deal with `inf`s or `nan`s in your arrays.

To test your node, you can launch the sim container with `kb_teleop` set to `True` in `sim.yaml`. Then in another `bash` session inside the sim container, launch the `teleop_twist_keyboard` node from `teleop_twist_keyboard` package for keyboard teleop. It should already be installed as a dependency of the simulator. After running the simulation, the keyboard teleop, and your safety node, use the reset tool for the simulation and drive the vehicle towards a wall.

Note the following topic names for your publishers and subscribers:

- `LaserScan`: /scan
- `Odometry`: /ego_racecar/odom, specifically, the longitudinal velocity of the vehicle can be found in `twist.twist.linear.x`
- `AckermannDriveStamped`: /drive

## V: Deliverables and Submission
You can implement this node in either C++ or Python. A skeleton package is already provided in the repo that you can use. Develop **directly in the simulation container** provided. Put your package in `/sim_ws/src` alongside the simulation package.
When following the instruction in the simulation repo, the repo directory will be mounted to the sim container. You can also add extra volumes mounted for your convenience when editing the files. For example, if you're using the `rocker` tool:

```bash
rocker --nvidia --x11 --volume .:/sim_ws/src/f1tenth_gym_ros --volume <path_to_your_package_on_host>:/sim_ws/src/safety_node -- f1tenth_gym_ros
```

Or if you're using `docker-compose`, add an extra line `- <path_to_your_package_on_host>:/sim_ws/src/safety_node` to your `volumes` field for the `sim` container.

Note that if you're using Windows, make sure your files have Unix style line endings. You can use `dos2unix` or have correct settings in your text editor.

**Deliverable 1**: After you're finished, update the entire skeleton package directory with your `safety_node` package and directly commit and push to the repo Github classroom created for you. Your commited code should start and run in simulation smoothly.

**Deliverable 2**: Make a screen cast of running your safety node in the simulation. Drive the car with keyboard teleop along the hallways of Levine, showing it doesn't brake when travelling straight in the hallway. You need to show that your safe node doesn't generate false positives. i.e. The car doesn't suddenly stop while travelling down the hallway. Then show the car driving towards a wall and braking correctly. Upload your video to YouTube (unlisted) and include a link to the video in **`SUBMISSION.md`**.

## VI: Grading Rubric
- Compilation: **30** Points
- Provided Video: **20** Points
- Correctly stops before collision: **30** Points
- Correctly calculates TTC: **10** Points
- Able to navigate through the hallway: **10** Points
