# F1TENTH gym environment ROS communication bridge
This is a containerized ROS communication bridge for F1TENTH gym environment.

This project is still under heavy developement.

# Overview

<img src="f1tenth_gym_ros.png" width="600">

# Installation
Before cloning this repo, you'll need to install Docker. Note that this environment is only tested on Ubuntu. You'll also need ROS on your host system.

Follow the instructions [here](https://docs.docker.com/install/linux/docker-ce/ubuntu/) to install Docker. A short tutorial can be found [here](https://docs.docker.com/get-started/) if you're not familiar with Docker.

Clone this repo into the ```src/``` directory in your worksapce, first build the docker image by:

```bash
$ cd f1tenth_gym_ros
$ sudo ./build_docker.sh
```

This will take around 5 minutes to build depending on your system

To run the containerized environment, start a docker container by:

```bash
$ sudo ./docker.sh
```

Next, in a new terminal in the host system, check everything is working by:
```bash
$ rostopic list
```

You should a see a few topics like the usual ```/rosout``` etc. And topics provided by the environment like ```/scan``` etc.

When you're creating your own launch file to launch your node, please include ```gym_bridge_host.launch``` in the ```launch``` directory in your own launch file by putting this line in your launch file:

```xml
<include file="$(find f1tenth_gym_ros)/launch/gym_bridge_host.launch"/>
```

An example agent launch file is in ```launch/agent_template.launch```

After you build your workspace after ```catkin_make```, you can run the agent template by running:

```bash
$ roslaunch f1tenth_gym_ros agent_template.launch
```

You should see an rviz window show up, showing the map, the two cars (ego is blud and opponent is orange), and the LaserScan of the ego car. The opponent is running pure pursuit around the track, and the ego agent is not moving.

# Available Topics

```/scan```: The ego agent's laser scan

```/odom```: The ego agent's odometry

```/opp_odom```: The opponent agent's odometry

```/map```: The map of the environment

```/race_info```: Information of the environment including both agents' elapsed runtimes, both agents' lap count, and both agents' collsion info.

# Developing and creating your own agent in ROS
A basic dummy agent node is provided in ```scripts/dummy_agent_node.py```. Launch your own node in your launch file, and don't forget to include ```gym_bridge_host.launch``` in your own launch file.

# TODO
- [x] Two-way comm tests
- [x] RobotModel state update
- [x] Some way to notify collision between agents
- [x] Some way to notify two cars finishing fixed number of laps
- [x] Since we have timer update instead of action stepping, what is the notion of 'done'?
- [x] Publish more topics on collsions, laptime, and done
- [x] Integrate example test agents
- [ ] Integrate competent racing agents (with random order when testing)
- [x] Fix mismatch between ray casted scan and robot model
- [ ] Add instruction in README for rebuilding image when remote repo updates