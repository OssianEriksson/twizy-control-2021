# twizy_control

ROS-package for sending car control messages to the Twizy's CAN bus.

**IMPORTANT**: Read through [Tests](#tests) before running tests!

# Installation

## Installation Prerequisites

This package requires some python packages that are not indexed by rospkg, meaning you have to install them manually.
(There is a process to get new dependicies indexed by rosdep, see [rosdep docs](https://docs.ros.org/en/independent/api/rosdep/html/contributing_rules.html), but I cant be bothered.)

Firstly you need to install `pip` (or `pip3` if you are using python3: replace `pip` with `pip3` in all the following):
```sh
sudo apt install python-pip
```

Next, install neccesary python packages:
```sh
pip install canlib
```

You also need to follow instructions on installing [Kvaser linux drivers and SDK](https://www.kvaser.com/linux-drivers-and-sdk-2/).

## local_keyboard

To run the `local_keyboard` node you also need to install pynput:
```sh
pip install pynput
```

If you are using linux, you may have to give yourself access to the `/dev/input` device files for the script to be able to access your keyboard.
On Debian and Ubuntu this can be done by adding yourself to the `input` group:
```sh
sudo usermod -a -G input $USER
```
Note that you have to log out and back in again for your addition to the group to take effect.

# Tests

**IMPORTANT**: Two things to note:

1. Don't run tests on the actual car, run tests on a separate computer.
   The tests are set up to immediately fail if they run on the real car by checking if TCP connections to GNSS recievers are available.
   Tests should not run on the real car as the the tests involves writing to the CAN-bus which if we are running on the real car and not on a virtual CAN device might cause the car to start moving about in a generally unsafe manner.

2. To run the tests you need to start a Kvaser virtual CAN device with at least two available channels.
   On linux two channels is the default, and the virtual device can be started with
   ```sh
   sudo /usr/sbin/virtualcan.sh start
   ```
   provided you have [installed the Kvaser linux SDK](#dependicies).

# ROS API

## control

Reads Twizy control reference values from ROS topic and publishes them to the Twizy's CAN bus.

### Subscribed Topics

`twizy_control` ([twizy_msgs/CarControl](../twizy_msgs/msg/CarControl.msg))

&emsp;Reference control (steering angle and speed) values for the Twizy.

### Parameters

`~channel` (`int`, default: 0)

&emsp;Kvaser CAN channel to write CAN frames to

`~timeout` (`int`, default: 150)

&emsp;Timeout (in ms) to wait for messages to be sent to the can bus before continuing

### Starting

Start the node with
```sh
rosrun twizy_control control
```

## local_keyboard

Reads key presses from the local keyboard (does not work over ssh) and publishes [twizy_msgs/CarControl](../twizy_msgs/msg/CarControl.msg) messages to drive the car.
This node uses the arrow keys to decide what messages to publish.

### Published Topics

`twizy_control` ([twizy_msgs/CarControl](../twizy_msgs/msg/CarControl.msg))

&emsp;Reference control (steering angle and speed) values for the Twizy from keyboard input

### Parameters

`~max_forward_speed` (`float`, default: 1.38)

&emsp;Reference forward speed to publish when up arrow is pressed (m/s)

`~max_backward_speed` (`float`, default: 1.38)

&emsp;Reference backward speed to publish when down arrow is pressed (m/s)

`~max_steering_angle` (`float`, default: 0.52)

&emsp;Reference steering angle to publish when either the left or right arrow key is pressed (radians)

### Starting

Start the node with
```sh
rosrun twizy_control local_keyboard
```