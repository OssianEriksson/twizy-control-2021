# twizy_control

ROS-package for sending car control messages to the Twizy's CAN bus.

**IMPORTANT**: Read through [Tests](#tests) before running tests!

# Dependicies

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

# Nodes

## control

Reads Twizy control reference values from ROS topic and publishes them to the Twizy's CAN bus.

### Starting

Start the node with
```sh
rosrun twizy_control control
```

Accepted parameters are
* *~channel* (int): Kvaser CAN channel to write CAN frames to, e.g. 0
* *~timeout* (int): Timeout (in ms) to wait for messages to be sent to the can bus before continuing, e.g. 150

### Subscribed Topics

#### twizy_control

Reference control (steering angle and speed) values for the Twizy.

* Data type: [twizy_control.msg.TwizyControl](msg/TwizyControl.msg)