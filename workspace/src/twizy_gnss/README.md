# twizy_gnss

ROS-package for interpreting and tranfsforming the Twizy's GNSS data.

# Installation

## Installation Prerequisites

## pip

Firstly you need to install `pip` (or `pip3` if you are using python3: replace `pip` with `pip3` in all the following):
```sh
sudo apt install python-pip
```

Finally, install neccesary python packages:
```sh
pip install utm numpy
```

## rosdep

**Warning**: Untested

```sh
rosdep install twizy_gnss
```

# ROS API

## GPS_left & GPS_right (old)

These two nodes read GNSS positioning data using the SBP protocol over a TCP socket and publishes it to two respective topics.
The source for the nodes are available in the [twizy_parking_2020 package](../twizy_parking_2020).
These nodes are here for testing purposes.
The nodes are made available under the names "gps_l" and "gps_r" respectively, both under the namespace "old".

### Published Topics

`GPS_left` & `GPS_right` ([std_msgs/String](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html), data formatted as "\<longitude\>,\<latitude\>" with both values in degrees)

&emsp;Contains positioning data from the respective GNSS reciever.

### Parameters

`~host_l` (`str`, default: "192.168.0.222")

&emsp;Host to recieve data from for left GNSS reciever

`~host_l` (`str`, default: "192.168.0.223")

&emsp;Host to recieve data from for right GNSS reciever

`~port_l` (`int`, default: 55555)

&emsp;Port to recieve data on for left GNSS reciever

`~port_r` (`int`, default: 55555)

&emsp;Port to recieve data on for right GNSS reciever

### Starting

Start the nodes with
```sh
roslaunch twizy_gnss old.launch
```

## twizy_gnss

This node reads GNSS data from the Twizy's left and right GNSS reciever and publishes a pose for use with localization.
The node listens for GNSS data of type [sensor_msgs/NavSatFix](https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html) on `/gnss/left/navsatfix_best_fix` and `/gnss/right/navsatfix_best_fix`.

### Published Topics

`gnss/pose` ([geometry_msgs/PoseWithCovarianceStamped ](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))

&emsp;Twizy's pose in the `gnss_center` reference frame.

### Starting

Start the nodes with
```sh
rosrun twizy_gnss twizy_gnss
```