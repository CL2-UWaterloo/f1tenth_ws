# F1TENTH gym environment ROS2 communication bridge
This is a containerized ROS communication bridge for the F1TENTH gym environment that turns it into a simulation in ROS2.

<!-- # Overview -->

<!-- <img src="f1tenth_gym_ros.png" width="600"> -->

<!-- # Different Benchmarks
In our virtual race, there will be three benchmark tasks. 

1. **Benchmark 1** is a single agent time trial without obstacle on the track. The objective is to achieve lower lap times. 
2. **Benchmark 2** is a single agent task with unknown obstacles in the map before hand. The objective is to finish laps without crashing. 
3. **Benchmark 3** is a task where two agents compete simultaneously on the same track. The objective is to finish a certain number of laps before the other agent.

We provide several branches for different benchmarks. On the **master** branch, the simulator is created for Benchmarks 1 & 2, where only a single agent (the ego agent) will spawn in the map. On the **multi_node** branch, the simulator is modified for Benchmark 3, where two agents will spawn in the map. We'll go over how these agents are controlled in a following section. -->

# Installation
<!---Before cloning this repo, you'll need to install Docker. Note that this environment is only tested on Ubuntu. You'll also need ROS on your host system. --->

**Supported System:**

- Ubuntu (tested on 20.04) with an NVIDIA gpu and nvidia-docker2 support
- We'll expand support to other systems soon

**Dependencies (with an nvidia gpu):**

- **Docker** Follow the instructions [here](https://docs.docker.com/install/linux/docker-ce/ubuntu/) to install Docker. A short tutorial can be found [here](https://docs.docker.com/get-started/) if you're not familiar with Docker. If you followed the post-installation steps you won't have to prepend your docker and docker-compose commands with sudo.
- **nvidia-docker2**, follow the instructions [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) if you have a support GPU. It is also possible to use Intel integrated graphics to forward the display, see details instructions from the Rocker repo.
- **rocker** [https://github.com/osrf/rocker](https://github.com/osrf/rocker). This is a tool developed by OSRF to run Docker images with local support injected. We use it for GUI forwarding.

**Dependencies (without an nvidia gpu):**

If your system does not support nvidia-docker2, noVNC will have to be used to forward the display.
- Again you'll need **Docker**. Follow the instruction above.
- Additionally you'll need **docker-compose**. Follow the instruction [here](https://docs.docker.com/compose/install/) to install docker-compose.
- You won't need nvidia-docker2 or rocker.

**Installation (with NVIDIA gpu):**

1. Clone this repo 
2. Build the docker image by:
```bash
$ cd f1tenth_gym_ros
$ docker build -t f1tenth_gym_ros -f Dockerfile .
```
3. To run the containerized environment, start a docker container by running the following. (example showned here with nvidia-docker support). By running this, the current directory that you're in (should be `f1tenth_gym_ros`) is mounted in the container at `/sim_ws/src/f1tenth_gym_ros`. Which means that the changes you make in the repo on the host system will also reflect in the container.
```bash
$ rocker --nvidia --x11 --volume .:/sim_ws/src/f1tenth_gym_ros -- f1tenth_gym_ros
``` 

**Installation (without NVIDIA gpu):**

1. Clone this repo 
2. Build the docker image by:
```bash
$ cd f1tenth_gym_ros
$ docker build -t f1tenth_gym_ros -f Dockerfile .
```
3. Bringup the novnc container and the sim container with docker-compose:
```bash
$ docker-compose up
``` 
4. In a separate terminal, run the following, and you'll have the same bash interface. Again tmux is available for convenience.
```bash
$ docker exec -it f1tenth_gym_ros_sim_1 /bin/bash
```
5. In your browser, navigate to [http://localhost:8080/vnc.html](http://localhost:8080/vnc.html), you should see the noVNC logo with the connect button. Click the connect button to connect to the session.

**Launching the Simulation**

1. tmux is also included in the contianer, so you can create multiple terminals in the same environment.
2. To launch the simulation, make sure you source both the ROS2 setup script and the local workspace setup script. Run the following in the bash session from the container:
```bash
$ source /opt/ros/foxy/setup.bash
$ source install/local_setup.bash
$ ros2 launch f1tenth_gym_ros gym_bridge.launch
```
A rviz window should pop up showing the simulation either on your host system or in the browser window depending on the display forwarding you chose.

<!-- > **When you're creating your own launch file to launch your node, please include ```gym_bridge_host.launch``` in the ```launch``` directory in your own launch file by putting this line in your launch file:**
> ```xml
> <include file="$(find f1tenth_gym_ros)/launch/gym_bridge_host.launch"/>
> ```

5. An example agent launch file is in ```launch/agent_template.launch```. After you build your workspace after ```catkin_make```, you can run the agent template by running:
```bash
$ roslaunch f1tenth_gym_ros agent_template.launch
```
You should see an rviz window show up, showing the map, the two cars (ego is blue and opponent is orange), and the LaserScan of the ego car. The opponent is running pure pursuit around the track, and the ego agent is not moving.
 -->
# Configuring the simulation
The configuration file for the simulation is at `f1tenth_gym_ros/config/sim.yaml`
TODO

# Available Topics for subscription
TODO
<!-- ```/scan```: The ego agent's laser scan

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
 -->