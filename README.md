# F1TENTH gym environment ROS communication bridge
This is a containerized ROS communication bridge for F1TENTH gym environment.

This project is still under heavy developement.

# Installation
Before cloning this repo, you'll need to install Docker. Note that this environment is only tested on Ubuntu. You'll also need ROS on your host system.

Follow the instructions [here](https://docs.docker.com/install/linux/docker-ce/ubuntu/) to install Docker. A short tutorial can be found [here](https://docs.docker.com/get-started/) if you're not familiar with Docker.

After cloning this repo, first build the docker image by:

```bash
$ cd f1tenth_gym_ros
$ sudo ./build_docker.sh
```

This will take around 10 minutes to build depending on your system

Then run start a docker container by:

```bash
$ sudo ./docker.sh
```

Next, in a new terminal in the host system, check everything is working by:
```bash
$ rostopic list
```

# TODO
1) Full system tests
2) Integrate example test agents