# F1TENTH gym environment ROS communication bridge
This is a containerized ROS communication bridge for F1TENTH gym environment.

This project is still under heavy developement.

# Overview

<img src="f1tenth_gym_ros.png" width="600">

# Different Benchmarks
In our virtual race, there will be three benchmark tasks. 

1. **Benchmark 1** is a single agent time trial without obstacle on the track. The objective is to achieve lower lap times. 
2. **Benchmark 2** is a single agent task with unknown obstacles in the map before hand. The objective is to finish laps without crashing. 
3. **Benchmark 3** is a task where two agents compete simultaneously on the same track. The objective is to finish a certain number of laps before the other agent.

We provide several branches for different benchmarks. On the **master** branch, the simulator is created for Benchmarks 1 & 2, where only a single agent (the ego agent) will spawn in the map. On the **multi_node** branch, the simulator is modified for Benchmark 3, where two agents will spawn in the map. We'll go over how these agents are controlled in a following section.

# Installation
<!---Before cloning this repo, you'll need to install Docker. Note that this environment is only tested on Ubuntu. You'll also need ROS on your host system. --->

**System Requirements:**
- Ubuntu (tested on 18.04)
- ROS (tested on Melodic)
- Docker (Follow the instructions [here](https://docs.docker.com/install/linux/docker-ce/ubuntu/) to install Docker. A short tutorial can be found [here](https://docs.docker.com/get-started/) if you're not familiar with Docker.)

1. Clone this repo into the ```src/``` directory in your workspace, 
2. Build the docker image by:
```bash
$ cd f1tenth_gym_ros
$ sudo ./build_docker.sh
```
This will take around 5 minutes to build depending on your system

3. To run the containerized environment, start a docker container by:
```bash
$ sudo ./docker.sh
```
4. Next, in a new terminal in the host system, check everything is working by:
```bash
$ rostopic list
```
You should a see a few topics like the usual ```/rosout``` etc. And topics provided by the environment like ```/scan``` etc.

> **When you're creating your own launch file to launch your node, please include ```gym_bridge_host.launch``` in the ```launch``` directory in your own launch file by putting this line in your launch file:**
> ```xml
> <include file="$(find f1tenth_gym_ros)/launch/gym_bridge_host.launch"/>
> ```

5. An example agent launch file is in ```launch/agent_template.launch```. After you build your workspace after ```catkin_make```, you can run the agent template by running:
```bash
$ roslaunch f1tenth_gym_ros agent_template.launch
```
You should see an rviz window show up, showing the map, the two cars (ego is blue and opponent is orange), and the LaserScan of the ego car. The opponent is running pure pursuit around the track, and the ego agent is not moving.

# Available Topics for subscription

```/scan```: The ego agent's laser scan

```/odom```: The ego agent's odometry

```/opp_odom```: The opponent agent's odometry

```/opp_scan```: The opponent agent's laser scan (only available on the multi_node branch)

```/map```: The map of the environment

```/race_info```: Information of the environment including both agents' elapsed runtimes, both agents' lap count, and both agents' collsion info. **Currently, the race ends after both agents finish two laps, so the elapsed times will stop increasing after both lap counts are > 2**

# Developing and creating your own agent in ROS
A basic dummy agent node is provided in ```scripts/dummy_agent_node.py```. Launch your own node in your launch file, and don't forget to include ```gym_bridge_host.launch``` in your own launch file.

On the **master** branch for single agent simulation, publish your drive message on the ```/drive``` topic using the AckermannDriveStamped message type. The simulation is stepped by a callback function subscribed to the drive topic.

On the **multi_node** branch for two-agent simulation, publish the ego agent's drive commands to ```/drive```, and the opponent agent's drive commands to ```/opp_drive```. At this point, we're not providing any agents built in for testing. A good way to start test your algorithms in this setting is to use another algorithm that you've created, or even the same algorithm.

# Changing maps
After you've ran the ```build_docker.sh``` script, you can copy the corresponding .yaml and image file into two directories: ```f1tenth_gym_ros/maps``` and ```f1tenth_gym_ros/f1tenth_gym/maps```. Then change the ```map_path``` and ```map_img_ext``` parameters in ```f1tenth_gym_ros/params.yaml``` to the corresponding paths. Lastly, change the ```map``` argument in ```f1tenth_gym_ros/launch/gym_bridge.launch``` to the new map.

After making all the changes, make sure you run ```build_docker.sh``` to rebuild the container.

You can find a collection of maps including the ones from past competitions here: https://github.com/f1tenth/f1tenth_simulator/tree/master/maps

# TODO
- [x] Two-way comm tests
- [x] RobotModel state update
- [x] Some way to notify collision between agents
- [x] Some way to notify two cars finishing fixed number of laps
- [x] Since we have timer update instead of action stepping, what is the notion of 'done'?
- [x] Publish more topics on collsions, laptime, and done
- [x] Integrate example test agents
- [ ] ~~Integrate competent racing agents (with random order when testing)~~
- [x] Fix mismatch between ray casted scan and robot model
- [ ] ~~Add instruction in README for rebuilding image when remote repo updates~~
- [ ] Handle env physics when collisions happen (agent-agent, agent-env)
- [ ] ~~Add some parameterization on racing scenarios~~
