[![Build Status](https://travis-ci.org/epfl-lasa/cpr_load_support.svg?branch=master)](https://travis-ci.org/epfl-lasa/cpr_load_support)

# Load-Support controller

This repository provides a controller for a robotic arm to recieve a heavy load (with known mass) at a expected height and lower it down to a user-friendly height and carry it around. Moreover, the robot uses speech to notify the human-user with the state of the controller.

While generic in its approach, this specific impelementation is done for clearpath mobile-robot with UR5 robotic-arm. The only requirment (for other robots) is to have an impedance controller implemented. For our case at lasa, we are using our impelementation at [ridgeback_ur5_controllers](https://github.com/epfl-lasa/ridgeback_ur5_controller/tree/devel).



---

## Compliation and build

Clone the repository intor your catkin source directory
```bash
$ cd ~/catkin_ws/src
$ git clone git@github.com:epfl-lasa/ridgeback_ur5_controller.git
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





checklist:

- how to compile and run


- add a figure for the control structure


- name of topics and variable

- pictures and videos


![alt text](doc/load_support_graph.png "State-dependency of the equlibrium w.r.t. the load-share")

![alt text](doc/load_support_algorithm.png "the code architecture.")

