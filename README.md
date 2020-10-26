# turtlebot_control
The purpose of this set of packages is to perform the linear control of the Turtlebot3 Waffle PI for trajectory tracking. The main algorithm is based on a linear proportional control which uses the linearized model of the robot. The entire repository should be cloned to ```~/catkin_ws/src```. To do so, navigate to ```~/catkin_ws/src``` and from a terminal and enter: ```git clone https://github.com/EDU4RDO-SH/turtlebot_control.git```. Then, navigate to ```~/catkin_ws``` and compile the code with the command: ```catkin_make```. If you are installing ROS for the first time, see the instructions [here](https://wiki.ros.org/kinetic/Installation/Ubuntu). This version has been created using the Bebop 2, ROS Kinetic, and Ubuntu 16.04.


## Setup

### 1. Turtlebot3 driver installation
First of all, it is necessary to set the driver up which is in charge of controlling the actuators and reading the sensors. If you want to test the controller in a real platform you have to configure the robot computer as well as the remote PC properly, a detailed description of the entire process can be found in the official site of the project Turtlebot3 in the [setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/setup/#setup) tab. In the other hand, if you wish to test the controller inside the Gazebo enviroment you need to download the corresponding packages, also from the official site in the [simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/setup/#setup) tab.


### 3. Clone the repository
Next, the entire repository should be cloned to ```~/catkin_ws/src``` by entering the command ```git clone https://github.com/EDU4RDO-SH/turtlebot_control.git``` and compile the code with ```catkin_make```.



## Examples

### bebop_takeoff_land
This program performs a simple takeoff, keeps the drone in hover mode 1 meter above the takeoff point for a few seconds, and then lands. For that, in a terminal run the Bebop driver:

```
roslaunch bebop_driver bebop_node.launch
```

In a new terminal execute the takeoff and land node:

```
roslaunch bebop_takeoff_land bebop_takeoff_land.launch
```

### bebop_teleop_joy
This package allows us to control the Bebop drone with the [Logitech F710](https://www.logitechg.com/en-us/products/gamepads/f710-wireless-gamepad.html) gamepad. This package depends on the [joy](https://wiki.ros.org/joy) node which enables the use of the gamepad with ROS by reading  the current state of each one of the joystick's buttons and axes through a ```sensor_msgs::Joy``` message. The following figure shows the configuration used.

<p align="center"><img src="https://i.imgur.com/JtsSjeA.png" width="700" /></p>

To run this program first in a terminal, run the Bebop driver:

```
roslaunch bebop_driver bebop_node.launch
```

In a second terminal execute the teleoperation node:

```
roslaunch bebop_teleop_joy bebop_teleop_joy.launch
```
To perform any action always keep the deadman button pressed. The first step is to send a takeoff command, and once in the air the drone will be ready to receive piloting commands. The default configuration allows the Bebop move at half its maximum speed, if you desire move the drone at its maximum speed you should edit the corresponding scaling parameter in the ```logF710.yaml``` file.



### bebop_odometry_example
This program subscribes to the ```/bebop/odom``` topic data published by the Bebop driver and populates a ```nav_msgs/Path``` message. The path generated is visualized in Rviz. To run the program, first execute the Bebop driver in a terminal:

```
roslaunch bebop_driver bebop_node.launch
```

We also make use of the ```bebop_teleop_joy``` node to move the drone around. For that, in a second terminal run:

```
roslaunch bebop_teleop_joy bebop_teleop_joy.launch
```

Finally, in a third terminal run the ```bebop_odometry_example``` node:

```
roslaunch bebop_odometry_example bebop_odometry_example.launch
```

As a result an Rviz window will pop up. Now yow can start moving the drone with the gamepad and see the path generated.  

<p align="center"><img src="https://i.imgur.com/mDa9Rgy.png" width="1000" /></p>

### bebop_gps_example
This package gets the GPS data: Latitude Longitude and Altitude provided by the drone through a ```sensor_msgs::NavSatFix```. The values are displayed on the Bebop's camera image. Additionally, the program monitors the percentage of charge of the battery, this let us to know when it is necessary to recharge it. To run the program first execute the Bebop driver in a terminal window:


```
roslaunch bebop_driver bebop_node.launch
```

Next, run the ```bebop_teleop_joy``` node in a new terminal window to pilot the drone:

```
roslaunch bebop_teleop_joy bebop_teleop_joy.launch
```

Finally, in a third terminal window execute the ```bebop_gps_example``` node:

```
roslaunch bebop_gps_example bebop_gps_example.launch
```

<p align="center"><img src="https://i.imgur.com/6YBFBrh.png" width="700" /></p>

It is important to note that GPS data will be available after Bebop's take off.

### bebop_control_inputs
This package subscribes to the ```/bebop/cmd_vel``` topic and converts the twist data whose range for all fields are ```[-1..1]``` into control variables to populate a ```mav_msgs::RollPitchYawrateThrust``` message. According to the [piloting](https://bebop-autonomy.readthedocs.io/en/latest/piloting.html#piloting) section in the [bebop_autonomy](https://bebop-autonomy.readthedocs.io/en/latest/#bebop-autonomy-ros-driver-for-parrot-bebop-drone-quadrocopter-1-0-2-0) official site, the *roll* and *pitch* angles depend on the value of ```max_tilt_angle``` parameter, the resultig *yaw rate* depends on the value of ```max_rot_speed``` parameter, and the *vertical speed* depends on the value of ```max_vert_speed``` parameter. The resulting data is published in the ```/bebop/cmd_ctrl```.


<p align="center"><img src="https://i.imgur.com/yY6nKXf.png" width="400" /></p>

Additionally, the current control inputs applied to the drone are displayed on the camera image. This program also monitors the percentage of charge of the battery. To run the program  first execute the driver in a terminal window:

```
roslaunch bebop_driver bebop_node.launch
```
Open a new terminal and run the ```bebop_teleop_joy``` node:

```
roslaunch bebop_teleop_joy bebop_teleop_joy.launch
```

Finally in a new terminal window run the ```bebop_control_inputs``` node:

```
roslaunch bebop_control_inputs bebop_control_inputs.launch
```

<p align="center"><img src="https://i.imgur.com/ZjAEOoj.png" width="700" /></p>
