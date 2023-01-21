# Lab 1: Docker and ROS 2

## Learning Goals

- Understanding Docker workflow
- Getting familiar with ROS 2 workflow inside Docker containers
- Understanding how to create nodes with publishers, subscribers
- Understanding ROS 2 package structure, files, dependenciees
- Creating launch files

## I. Overview

The goal of this lab is to get you familiar with the ROS 2 workflow inside containers. You'll have the option to complete the coding segment of this assignment in either Python or C++. However, we highly recommend trying out both since this will be the easiest assignment to get started on a new language, and the workflow in these two languages are slightly different in ROS2 and it's beneficial to understand both.

In this lab, it'll be helpful to read these tutorials if you're stuck:

[https://docs.ros.org/en/foxy/Tutorials.html](https://docs.ros.org/en/foxy/Tutorials.html)

[https://roboticsbackend.com/category/ros2/](https://roboticsbackend.com/category/ros2/)

## II. Getting ready

First, install Docker on your system following the instructions here: [https://docs.docker.com/get-docker/](https://docs.docker.com/get-docker/). When you're working with simulation only, we'll make sure the assignment can be completed on all three platforms. When working on the car, everything will be in Linux. Note that you should also complete the post installation steps on some platforms, or you'll need to call docker with ```sudo```.

Next, start a container with a bind mount to your workspace directory on your host system inside this repo by:

```bash
docker run -it -v <absolute_path_to_this_repo>/lab1_ws/src/:/lab1_ws/src/ --name f1tenth_lab1 ros:foxy
```

This will create a workspace directory on the host at `<absolute_path_to_this_repo>/lab1_ws/src`. It'll create the container based on the official ROS 2 Foxy image, and give the container a name `f1tenth_lab1`. You'll then have access to a terminal inside the container.

`tmux` is recommended when you're working inside a container. It could be installed in the container via: `apt update && apt install tmux`. `tmux` allows you to have multiple `bash` session in the same terminal window. This will be very convenient working inside containers. A quick reference on how to use tmux can be found [here](https://tmuxcheatsheet.com/).  You can start a session with `tmux`. Then you can call different `tmux` commands by pressing `ctrl+B` first and then the corresponding key. For example, to add a new window, press `ctrl+B` first and release and press `c` to create a new window. You can also move around with `ctrl+B` then `n` or `p`.

## III: ROS 2 in Docker

Now that we have the access to a ROS 2 container, let's test out the basic ROS 2 commands. In the terminal, run:

```bash
ros2 topic list
```
You should see two topics listed:
```bash
/parameter_events
/rosout
```

If you need multiple terminals inside the container, use `tmux`.

## IV: Creating a Package
**Deliverable 1**: create a package named `lab1_pkg` in the workspace we created. The package needs to meet these criteria:
- The package supports both `Python` and `C++`.
- The package needs to have the `ackermann_msgs` dependency.
- Both of these can be done by declaring the correct dependencies in `package.xml`.
- If declared properly the depencies could be installed using `rosdep`.
- Your package folder should be neat. You shouldn't have multiple 'src' folders or unnecessary 'install' or 'build' folders.

## V: Creating nodes with publishers and subscribers
**Deliverable 2**: create two nodes in the package we just created. You can use either `Python` or `C++` for these nodes.

The first node will be named `talker.cpp` or `talker.py` and needs to meet these criteria:
- `talker` listens to two ROS parameters `v` and `d`.
- `talker` publishes an `AckermannDriveStamped` message with the `speed` field equal to the `v` parameter and `steering_angle` field equal to the `d` parameter, and to a topic named `drive`.
- `talker` publishes as fast as possible.
- To test node, set the two ROS parameters through command line, a launch file, or a yaml file.

The second node will be named `relay.cpp` or `relay.py` and needs to meet these criteria:
- `relay` subscribes to the `drive` topic.
- In the subscriber callback, take the speed and steering angle from the incoming message, multiply both by 3, and publish the new values via another `AckermannDriveStamped` message to a topic named `drive_relay`.

## VI: Creating a launch file and a parameter file
**Deliverable 3**: create a launch file `lab1_launch.py` that launches both of the nodes we've created. If you want, you could also set the parameter for the `talker` node in this launch file.

## VII: Tagging and pushing your image to Docker Hub
You can use Docker Hub to easily share container images with your team. You might find it useful in the future when you're working in a team. For a quickstart guide on how to tag and push your images, see [https://docs.docker.com/docker-hub/](https://docs.docker.com/docker-hub/).

**Deliverable 4**: register for a Docker ID and create a public repo on Docker Hub named `f1tenth_lab1`. Tag the container that you currently have that includes the workspace, packages, and nodes created as `latest` and push to Docker Hub. You can see a detailed guide here [https://docs.docker.com/docker-hub/repos/](https://docs.docker.com/docker-hub/repos/) if you're stuck.

## VIII: ROS 2 commands

After you've finished all the deliverables, launch the two nodes and test out these ROS 2 commands:
```bash
ros2 topic list
ros2 topic info drive
ros2 topic echo drive
ros2 node list
ros2 node info talker
ros2 node info relay
```

## IX: Deliverables and Submission
In addition to the three deliverables described in this document, fill in the answers to the questions listed in **`SUBMISSION.md`**.

We'll be using Github classroom throughout the semester to manage submissions for lab assignments. After you're finished, directly commit and push to the repo Github classroom created for you.

## X: Grading Rubric
- Using ROS 2 inside a Docker container and pushing resulting image to Docker Hub: **20** Points
- Correctly creating the package: **20** Points
- Correctly creating the nodes: **20** Points
- Correctly creating the launch file: **20** Points
- Written questions: **5** Points each
