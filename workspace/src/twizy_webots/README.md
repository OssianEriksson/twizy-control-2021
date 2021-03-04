# twizy_webots

ROS-package for simulating the Twizy in the webots simulation program.
This package makes use of the [built in webots ROS controller](https://cyberbotics.com/doc/guide/using-ros#standard-ros-controller).

# Installation

Install [webots_ros](http://wiki.ros.org/webots_ros#Installation) and it's dependicies.

# ROS API

## control

This is a node for interfacing with the webots simulation.
It mainly reads [twizy_msgs/CarControl](../twizy_msgs/msg/CarControl.msg) messages and sends the contained reference values to the simulation.

### Subscribed Topics

`twizy_control` ([twizy_msgs/CarControl](../twizy_msgs/msg/CarControl.msg))

&emsp;Reference control (steering angle and speed) values for the Twizy to pass on to the webots simulation

### Parameters

TODODOD

### Starting

Start the node with
```sh
rosrun twizy_webots control
```