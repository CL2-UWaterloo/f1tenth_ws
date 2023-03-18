# f1tenth_ws
This a repository that contains ready-to-run autonomous racing packages for the [F1TENTH](https://f1tenth.org/) on ROS2 Foxy. It can be directly be deployed on the physical car. We've also included launch and config files for the [simulation environment](https://github.com/f1tenth/f1tenth_gym_ros), which uses slightly different topics for odometry.

Below is a demo of the car running the code from this repository in the E7 building at the University of Waterloo at a top speed of ~25km/h, record on March 13th 2023.

<iframe width="560" height="315" src="https://www.youtube.com/embed/2Xz8tVUvkdI" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

## Software Stack Overview
**Our current software stack consists of**
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) for **mapping** (reference the following [slides](https://docs.google.com/presentation/d/1DP2F9l-yHe9gQobk2CzYduk6KR5QtDCp7sLsxqR2fag/edit#slide=id.g115c48c178d_0_1) for running it on the physical car)
- [Particle Filter](./src/particle_filter/) for **localization** $\rightarrow$ `src/particle_filter`
- [Pure Pursuit](./src/pure_pursuit/) for **waypoint following** (planning + control) $\rightarrow$ `src/pure_pursuit`
- [RRT](./src/rrt) for a pure pursuit algorithm that includes **dynamic obstacle avoidance** (slightly slower) $\rightarrow$ `src/rrt`

Racing lines are generated through the [Cl2-UWaterloo/Raceline-Optimization](https://github.com/CL2-UWaterloo/Raceline-Optimization) repository.

**Other algorithms that are not used, but are in this repository include**
- [Waypoint Generator](./src/waypoint_generator/) for manually generating waypoints in simulation $\rightarrow$ `src/waypoint_generator` (this has been replaced with a script that automatically generates optimal racelines given a map)
- A [PID controller](./src/wall_follow/) for staying at a constant distance to the wall $\rightarrow$ `src/wall_follow`
- [Scan matching](./src/scan_matching) $\rightarrow$ `src/scan_matching` (To be completed)
- [gap_follow](./src/gap_follow) $\rightarrow$ `src/gap_follow` (To be completed)

## High-Level Usage Guide
These are the high level steps followed to get the F1TENTH driving in a new location:

1. Run SLAM on the physical car to generate a map with `slam_toolbox`
2. Clean up map in Photoshop, and generate a racing line using the [Cl2-UWaterloo/Raceline-Optimization](https://github.com/CL2-UWaterloo/Raceline-Optimization) repository.
3. Store the racing lines under `src/pure_pursuit/racelines/` and `src/rrt/racelines/` (for dynamic obstacle avoidance)
4. Run `particle_filter` with the new map to localize the car properly
5. Run `pure_pursuit` or `rrt` to follow the racing line. Make sure to incrementally increase the `velocity_profile` inside the [config.yaml](./src/pure_pursuit/config/config.yaml) file.

You can consult these accompanying notes: <https://stevengong.co/notes/F1TENTH-Field-Usage>, which shows every command used to get code working the physical car. Note that they are mainly written for our own personal reference, so the paths will be different in your setup.

If you feel confused / stuck by all of this, you might want to start with following the [F1TENTH Course](https://docs.google.com/spreadsheets/d/1kAd0bf6nc1OVi_4IP1P3-H6PPU97hLjqW8d0mTLCsxg/edit#gid=29915317), where they actually teach why these algorithms are needed, and how they are implemented. The [official documentation](https://f1tenth.readthedocs.io/en/foxy_test/) describes how to setup the hardware and basic software for the car.

## Running Simulation
If you want to run these nodes inside the simulation environment, you need to clone the [simulation repository](https://github.com/f1tenth/f1tenth_gym_ros). Then, you should mount this repository's packages to the `docker-compose.yml` file:

```
- INSERT_PATH_HERE/f1tenth_ws/src:/sim_ws/src
```

Each package can then be built inside the simulation environment.

## Next Steps
**A non-exhaustive list of things we want to do in the future, include**

- Running SLAM + pure pursuit on the fly, without having to do the offline computation. Something like [this](https://www.youtube.com/watch?v=aCDPwZZm9C4&ab_channel=AMZFormulaStudent) would be incredible

## About Us
The Control, Learning and Logic (CL2) group at the University of Waterloo works on reseearch that aims to develop methods for reliable decision-making of autonomous systems in the wild, led by professor Yash Vardhan Pant. Current members working on the F1TENTH is composed of Steven Gong, Oluwatofolafun Damilola Opeoluwa-Calebs, and Soham Lakhi.
