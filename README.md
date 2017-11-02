[![Build Status](https://travis-ci.org/epfl-lasa/cpr_load_support.svg?branch=master)](https://travis-ci.org/epfl-lasa/cpr_load_support)

# Load-Support controller

This repository provides a controller for a robotic arm to receive a heavy load (with known mass) at a expected height and lower it down to a user-friendly height and carry it around. Moreover, the robot uses speech to notify the human-user with the state of the controller.

While generic in its approach, this specific implementation is done for clearpath mobile-robot with UR5 robotic-arm. The only requirement (for other robots) is to have an impedance controller implemented. For our case at LASA, we are using our implementation at [ridgeback_ur5_controllers](https://github.com/epfl-lasa/ridgeback_ur5_controller/tree/devel).

## Demonstration
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/AB7B2HuQdQ0/0.jpg)](https://youtu.be/AB7B2HuQdQ0)



---
## The control architecture

The architecture of the load-support controller (which is implemented in [LoadSupportController.cpp](https://github.com/epfl-lasa/cpr_load_support/blob/master/src/LoadSupportController.cpp)) is depicted below.

![alt text](doc/load_support_algorithm.png "the code architecture.")

### input-output
As you can see, the input signal to this code is the external force (F<sub>ext</sub>) and vision system information (published on ROS as transformation). The code outputs a desired equilibrium point and a control wrench to the admittance controller of the robot. The setting of these communication can be found in [the launch file](https://github.com/epfl-lasa/cpr_load_support/blob/master/launch/cpr_load_support.launch).

### Load-share computation
Based on the external forces on the z-axis (F<sub>ext,z</sub>) and the given mass for the object (M), the robot computes the load-share (noted as alpha). 

### Equilibrium computation
Based on the load-share and the location of the marker (which might be detached from the object), the robot adapts its equilibrium point. The computation of the equilibrium point is done separately for z-xis and the x-y-plane. In the z-axis, the robot computes the equilibrium point using the following function (noted as g).
![alt text](doc/load_support_graph.png "State-dependency of the equilibrium w.r.t. the load-share")
This means, if the robot receives loads more than a certain amount (alpha<sub>trigger</sub>) it will bring it down. Also, the robot holds the object at lower-level for load-share higher than (alpha<sub>final</sub>). In this graph, (Z<sub>ceiling</sub>) is the expected height to receive the object and (Z<sub>level</sub>) is the final expected height. These parameters can be set from [the launch file](https://github.com/epfl-lasa/cpr_load_support/blob/master/launch/cpr_load_support.launch).



To compute the desired equilibrium in xy-plane, the robot considers the location of the object and load-share as follows. 
* If the maker is far, the robot ignore it and goes back to its resting equilibrium (whether it has the object or not).
* If the marker is in reach, the robot set the x-y of the equilibrium as the x-y of the object (filtered and limited by the workspace).
    - if the load-share is low, the robot tracks the marker in order to receive the object.
    - if the load-share is high, the robot follows the marker in order to carry the object for the human-user.
* If the maker is very close the end-effector of the robot and the load-share is high enough, the robot tries to slowly bring the object to its resting equilibrium point. 


### Sound play
The states computed above are played as sound using [ros-sound-play](http://wiki.ros.org/sound_play). The robot also take two values for the timing of the speaking. "TALKING_RATE_MAX" (in seconds) forces the robot to pause between statements. Moreover, the robot only talks when there is new statement. On the other hand, "TALKING_RATE_MIN" (in seconds) forces the robot not stay silent more than this value (even the statement is repetitive).


### Weight compensation
In the Z-axis the robot cancel the effect of weight on the admittance control by sending its negated values as control wrench. However, this value cannot be exceed from F<sub>max</sub> than can be set from  [the launch file](https://github.com/epfl-lasa/cpr_load_support/blob/master/launch/cpr_load_support.launch).

The robot also cancels a portion of side-way forces. This portion is equal to untransformed weight. Thus, this function (noted as D) can be seen as a adaptive-dead-zone. The rationale behind this dead-zone is compensate for side-way forces that are actually created by the object that is tilted. Once the robot is supporting the whole weight, the compliant behavior is back to the one exhibited by the admittance controller.


### Summary of variables


| Variable      | Parameter                         |
|---------------|-----------------------------------|
| WRENCH_EXTERNAL_TOPIC    | The topic to receive the external force from          |
| WRENCH_CONTROL_TOPIC     | The topic to send control wrench to admittance controller       |
| DESIRED_EQUILIBRIUM_TOPIC| The topic to send the desired equilibrium point to the admittance controller   |
| FRAME_ARM_BASE           | The frame id for the base of the arm |
| FRAME_ARM_EE             | The frame id for the end-effector of the arm     |
| FRAME_OBJECT             | The frame id for the object   |
| M_object             | The mass of the object to be transferred  |
| MAX_WEIGHT             | The maximum weight that is compensated    |
| Z_ceiling             | The height where the object is expected to be received   |
| Z_level             | The height where the object is expected to be brought down  |
| LOADSHARE_TRIGGER             | The load-share that triggers the robot to bring the object down   |
| LOADSHARE_FINAL             | The required load-share to reach lowest height  |
| LOADSHARE_CONTACT             | The load-share that the robot assume it has contact with the object   |
| TALKING_RATE_MAX             | The maximum speaking rate (in seconds)  |
| LOADSHARE_CONTACT             | The minimum speaking rate (in seconds)   |







---

## Compilation and build

Clone the repository into your catkin source directory
```bash
$ cd ~/catkin_ws/src
$ git clone git@github.com:epfl-lasa/cpr_load_support.git
```

Get the package dependencies using rosdep
```bash
$ rosdep install -y --from-paths src --ignore-src --rosdistro indigo
```

Finally complie
```bash
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
```


## Running the controller

In order to run this controller for ridgeback robot and LASA-vision system you need to run the followings:

ssh to the robot and run the following:
```bash
$ roslaunch cpr_bringup cpr_bringup.launch sim:=false
```
To learn how to ssh to the robot, check the wiki of the lab.


For the vision system, make sure your ros-mocap configurations (e.g., [this](https://github.com/epfl-lasa/ridgeback_ur5_controller/tree/devel/cpr_mocap_tracking/config))
```bash
$ roslaunch cpr_mocap_tracking cpr_object_tracking.launch
```
You can check the in rviz if a tf for the object is published w.r.t. to robot frames.


For the admittance controller:
```bash
$ roslaunch admittance_control admittance_controller_real.launch
```
Also make sure about your [admittance configurations](https://github.com/epfl-lasa/ridgeback_ur5_controller/blob/devel/admittance_control/config/admittance_params_real.yaml)

Finally, run the load-support controller
```bash
$ roslaunch cpr_load_support cpr_load_support.launch
```
