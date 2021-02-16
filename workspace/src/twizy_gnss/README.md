# twizy_gnss

ROS-package for reading and publishing GNSS positioning data.
This package borrows a lot code, topic anc parameter names from [ethz_piksi_ros](https://github.com/ethz-asl/ethz_piksi_ros/tree/master/piksi_multi_rtk_ros).
We are writing our own code heare instead of just using that existing package because we are looking to upgrade to ROS 2 soon and that package isn't available for ROS 2 yet.
However if it does become available at a later date it should not be too hard to tear out the current piksi code and replace it with that package, since e.g. topics and parameters are using the same names.

# Dependicies

This package requires some python packages that are not indexed by rospkg, meaning you have to install them manually.
(There is a process to get new dependicies indexed by rosdep, see [rosdep docs](https://docs.ros.org/en/independent/api/rosdep/html/contributing_rules.html), but I cant be bothered.)

Firstly you need to install `pip` (or `pip3` if you are using python3: replace `pip` with `pip3` in all the following):
```sh
sudo apt install python-pip
```

Finally, install neccesary python packages:
```sh
pip install sbp utm numpy
```

# Nodes

## piksi

Reads native data from a GNSS reciever and publishes it.
Very similar to piksi node from [ethz_piksi_ros](https://github.com/ethz-asl/ethz_piksi_ros/tree/master/piksi_multi_rtk_ros) (should hopefully be a drop in replacement).

### Starting

To use the SBP protocol over a TCP socket as a GNSS data source, start the node using
```
rosrun twizy_gnss piksi
```

Accepted parameters are
* *~tcp_addr* (str): Host to recieve data from, e.g. "127.0.0.1" or "www.example.com"
* *~tcp_port* (int): Port to recieve data on, e.g. 12345

### Published Topics

#### ~navsatfix_best_fix

Positioning data from GNSS reciever.
LLH stands for Latitude Longitude Height.

* Data type: [sensor_msgs/NavSatFix](https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html)

## GPS_left & GPS_right (old)

These two nodes read GNSS positioning data using the SBP protocol over a TCP socket and publishes it to two respective topics.
The source for the nodes are available in the [twizy_parking_2020 package](../twizy_parking_2020).
These nodes are here for testing purposes.
The nodes are made available under the names "gps_l" and "gps_r" respectively, both under the namespace "old".

### Starting

Start the nodes with
```sh
roslaunch twizy_gnss old.launch
```

Accepted arguments are
* *host_l* & *host_r* (str): Host to recieve data from for the respecitve node, e.g. "127.0.0.1" or "www.example.com"
* *port_l* & *port_r* (int): Port to recieve data on for the respetive node, e.g. 12345

### Published Topics

#### ~GPS_left & ~GPS_right

Using the default namespaces and node names and namespaces, the topics will be available under /old/gps_l/GPS_left and /old/gps_r/GPS_right respectively.
Contains positioning data from GNSS reciever.

* Data type: [std_msgs/String](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html) (data formatted as "\<longitude\>,\<latitude\>" with both values in degrees)