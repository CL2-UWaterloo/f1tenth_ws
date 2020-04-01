# F1TENTH gym environment ROS communication bridge
This is a containerized ROS communication bridge for F1TENTH gym environment.

This project is still under heavy developement.

# Installation
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