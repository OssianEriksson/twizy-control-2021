# gnss

ROS-package for reading and publishing GNSS positioning data.

# Dependicies

This package requires some python packages that are not indexed by rospkg, meaning you have to install them manually.
(There is a process to get new dependicies indexed by rosdep, see [rosdep docs](https://docs.ros.org/en/independent/api/rosdep/html/contributing_rules.html), but I cant be bothered.)

Firstly you need to install `pip` (or `pip3` if you are using python3: replace `pip` with `pip3` in all the following):
```sh
sudo apt install python-pip
```

Finally, install neccesary python packages:
```sh
pip install sbp
```

# Nodes

## gnss

Reads native data from a GNSS reciever and publishes it.

### Starting

Depending on the native connection to your GNSS reciever, you will want to choose the corresponding script to start the node.

#### TCP-SBP

To use the SBP protocol over a TCP socket as a GNSS data source, start the node using
```
rosrun gnss tcp
```

Accepted parameters are
* *~host* (str): Host to recieve data from, e.g. "127.0.0.1" or "www.example.com"
* *~port* (int): Port to recieve data on, e.g. 12345

### Published Topics

#### ~gnss_llh

Positioning data from GNSS reciever.
LLH stands for Latitude Longitude Height.

* Data type: [gnss.msg.GNSSLatLongHeight](msg/GNSSLatLongHeight.msg)

## GPS_left & GPS_right (old)

These two nodes read GNSS positioning data using the SBP protocol over a TCP socket and publishes it to two respective topics.
The source for the nodes are available in the [twizy_parking_2020 package](../twizy_parking_2020).
These nodes are here for testing purposes.
The nodes are made available under the names "gps_l" and "gps_r" respectively, both under the namespace "old".

### Starting

Start the nodes with
```sh
roslaunch gnss old.launch
```

Accepted arguments are
* *host_l* & *host_r* (str): Host to recieve data from for the respecitve node, e.g. "127.0.0.1" or "www.example.com"
* *port_l* & *port_r* (int): Port to recieve data on for the respetive node, e.g. 12345

### Published Topics

#### ~GPS_left & ~GPS_right

Using the default namespaces and node names and namespaces, the topics will be available under /old/gps_l/GPS_left and /old/gps_r/GPS_right respectively.
Contains positioning data from GNSS reciever.

* Data type: std_msgs.msg.String (data formatted as "\<longitude\>,\<latitude\>" with both values in degrees)