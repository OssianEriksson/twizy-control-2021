# piksi_multi_rtk

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
pip install sbp
```

# ROS API

## piksi

Reads native data from a GNSS reciever and publishes it.
Very similar to piksi node from [ethz_piksi_ros](https://github.com/ethz-asl/ethz_piksi_ros/tree/master/piksi_multi_rtk_ros) (should hopefully be a drop in replacement).

### Published Topics

`~navsatfix_best_fix` ([sensor_msgs/NavSatFix](https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html))

&emsp;Positioning data from GNSS reciever. 
      LLH stands for Latitude Longitude Height.

### Parameters

`~tcp_addr` (`str`, default: "192.168.0.222")

&emsp;Host to recieve data from

`~tcp_port` (`int`, default: 55555)

&emsp;Port to recieve data on

### Starting

To use the SBP protocol over a TCP socket as a GNSS data source, start the node using
```
rosrun piksi_multi_rtk piksi
```
