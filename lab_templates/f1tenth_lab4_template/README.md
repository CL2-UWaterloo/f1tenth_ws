# Lab 4: Follow the Gap

## I. Learning Goals

- Reactive methods for obstacle avoidance

## II. Overview

In this lab, you will implement a reactive algorithm for obstacle avoidance. While the base starter code defines an implementation of the F1TENTH Follow the Gap Algorithm, you are allowed to submit in C++, and encouraged to try different reactive algorithms or a combination of several. In total, the python code for the algorithm is only about 120 lines.

## III. Review of F1TENTH Follow the Gap

The lecture slides on F1TENTH Follow the gap is the best visual resource for understanding every step of the algorithm. However, the steps are outlined over here:

1. Obtain laser scans and preprocess them.
2. Find the closest point in the LiDAR ranges array.
3. Draw a safety bubble around this closest point and set all points inside this bubble to 0. All other non-zero points are now considered “gaps” or “free space”.
4. Find the max length “gap”, in other words, the largest number of consecutive non-zero elements in your ranges array.
5. Find the best goal point in this gap. Naively, this could be the furthest point away in your gap, but you can probably go faster if you follow the “Better Idea” method as described in lecture.
6. Actuate the car to move towards this goal point by publishing an `AckermannDriveStamped` to the /drive topic.

### IV. Implementation

Implement a gap follow algorithm to make the car drive autonomously around the Levine Hall map. You can implement this node in either C++ or Python. There are two extra test maps `levine_blocked.png`, which is empty, and `levine_obs.png`, which has obstacles that are relatively hard to navigate through for you to evaluate your code on.

To change the map in the simulation, add the included `.png` and `.yaml` map files to `f1tenth_gym_ros/maps` directory. Then, change `f1tenth_gym_ros/config/sim.yaml` to use your desired map.

### V. Deliverables and Submission

**Deliverable 1**: After you're finished, update the entire skeleton package directory with your `gap_follow` package and directly commit and push to the repo Github classroom created for you. Your commited code should start and run in simulation smoothly.

**Deliverable 2**: Make a screen cast of running your reactive node in the simulation. Include a link to the video on YouTube in **`SUBMISSION.md`**. The basic requirement is that your car should be able to navigate entire loops in `levine_blocked` map, and through at least most of the obstacles in `levine_obs` map. Make screen casts on both maps.

### VI. Grading Rubric

- Compilation: **10** Points
- Implemented Find-Max Gap: **40** Points
- Implemented Find best point: **30** Points
- Levine blocked Video: **10** Points
- Levine obstacles Video: **10** Points

### VII. Extra Resources

UNC Follow the Gap Video: https://youtu.be/ctTJHueaTcY
