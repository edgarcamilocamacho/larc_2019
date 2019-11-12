# SIOC: Sistema Inteligente Organizador de Contenedores (Intelligent Container Organizer System)

This repository contains the code of the robot SIOC (Intelligent Container Organizer System), used for the participation of the Universidad Santo Tomás, Colombia, in the Latin American Robotics Competition, in Rio Grande, Brasil

## Requirements:

SIOC:
* Ubuntu 16.04
* ROS Kinetic
* Dynamixel ROS libraries: `ros-kinetic-dynamixel-controllers, ros-kinetic-dynamixel-driver, ros-kinetic-dynamixel-motor, ros-kinetic-dynamixel-msgs`
* OpenCV 4.1.1
* PyQt

## Installing:

Create the workspace, clone the repository and build:

``` bash
$ mkdir -p ~/ros/larc_ws/scr
$ cd ~/ros/larc_ws/src/
$ git clone https://github.com/edgarcamilocamacho/larc_2019
$ cd ..
$ catkin_make
```

To use, ensure to initialize the workspace:

``` bash
$ mkdir -p ~/ros/larc_ws/
$ source devel/setup.bash
```

## Starting the robot:

Ros core:

``` bash
$ roscore
```

***Motion nodes***, it starts Dynamixel controllers and MoveIt model:

``` bash
$ roslaunch larc_movement movement.launch
```

***Motion nodes***, it starts vision nodes and settings publishers:

``` bash
$ roslaunch larc_vision larc_vision.launch
```

***Finite States Machine***, it starts the FSM node that controls the high level functions:

``` bash
$ rosrun larc_control larc_fsm.py
```

## Controller:

To control the robot manually and visualize the camera image and masks, start the controller node:

``` bash
$ rosrun ros_control larc_controller.py
```

## Settings:

In order to adjust the vision settings, start the settings gui node:

``` bash
$ rosrun ros_settings larc_settings_gui.py
```



