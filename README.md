# turtlebot_control
The purpose of this set of packages is to perform the linear control of the Turtlebot3 Waffle PI for trajectory tracking. The main algorithm is based on a linear proportional control which uses the linearized model of the robot. The ```pub_desired_states``` package computes the desired states taking into account physical and actuator limitations. The ```turtlebot_linear_control``` package subscribes to the ```/odom``` topic, computes the adequate control signals and publish them to the ```cmd_vel``` topic.  


<p align="center"><img src="https://i.imgur.com/rMxmCAu.png" width="800" /></p>

## Setup

### 1. Turtlebot3 driver installation
First of all, it is necessary to set the driver up which is in charge of controlling the actuators and reading the sensors. If you want to test the controller in a real platform you have to configure the robot computer as well as the remote PC properly, a detailed description of the entire process can be found in the official site of the project Turtlebot3 in the [setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/setup/#setup) tab. In the other hand, if you wish to test the controller inside the Gazebo enviroment you need to download the corresponding packages, also from the official site in the [simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#ros-1-simulation) tab.


### 2. Clone the repository
The entire repository should be cloned to ```~/catkin_ws/src``` by entering the command ```git clone https://github.com/EDU4RDO-SH/turtlebot_control.git``` and compile the code with ```catkin_make```. If you are installing ROS for the first time, see the instructions [here](https://wiki.ros.org/kinetic/Installation/Ubuntu). This version has been created using the Bebop 2, ROS Kinetic, and Ubuntu 16.04.



## Usage

### Real platform
First, execute the robot driver in a terminal:


```
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

Then launch the control algorithm in another terminal window, in that moment the robot will start moving following the reference.
```
$ roslaunch turtlebot_linear_control linear_control_path1.launch
```

<p align="center"><img src="https://i.imgur.com/ke34wZ5.png" width="800" /></p>

The control performance can be seen in the next [video](https://www.youtube.com/watch?v=gjtTbT0YgIY).



### Simulation
If you want to test the control algorithm in simulation execute the vitual robot with the following commands.

```
$ export TURTLEBOT3_MODEL=waffle_pi
$ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```

Next, launch the controller.

```
$ roslaunch turtlebot_linear_control linear_control_path1.launch
```

Similary the robot will track the desired trajectory.


<p align="center"><img src="https://i.imgur.com/fLj2PQn.png" width="1000" /></p>
