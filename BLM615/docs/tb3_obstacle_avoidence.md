# TurtleBot3 Obstacle Avoidance

## Pre-requisites
- Ubuntu 18.04
- ROS Melodic Morenia
- Python 2
- Gazebo
- Required Turtlebot3 Packages

## Install TurtleBot3 Packages

```shell
$ sudo apt-get install ros-melodic-turtlebot3-msgs
$ sudo apt-get install ros-melodic-turtlebot3
```

## Install TurtleBot3 Simulation Package

```shell
$ cd ~/catkin_ws/src/
$ git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/catkin_ws && catkin_make
```

## Launch the Simulation Environment

```shell
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

## Demo

This is a Python code for implementing obstacle avoidance for a robot using ROS (Robot Operating System). The code defines a class `ObstacleAvoidance` that subscribes to the laser scan topic `/scan`, which is published by the Lidar sensor on the robot. The class also initializes a publisher to the topic `/cmd_vel`, which sends velocity commands to the robot.

In the `__init__` method of the class, the ROS node is initialized, and the subscriber and publisher are created. The `laser_callback` method is called every time a new laser scan message is received. In the `laser_callback` method, the distances to obstacles in front, right, and left of the robot are calculated using the laser scan data. If the distance to an obstacle in front is less than 0.3 meters, the robot's linear velocity is set to zero, and its angular velocity is set to turn away from the side with the closer obstacle. The robot will also stop and print the distance information to the ROS log. If there are no obstacles within 0.3 meters in front of the robot, it will continue moving forward with a linear velocity of 0.2 meters per second.

```shell
$ ./tb3_obstacle_avoidence.py
```