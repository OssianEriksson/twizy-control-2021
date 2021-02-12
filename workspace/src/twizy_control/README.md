# twizy_control

ROS-package for sending car control messages to the Twizy's CAN bus.

# Dependicies

This package requires some python packages that are not indexed by rospkg, meaning you have to install them manually.
(There is a process to get new dependicies indexed by rosdep, see [rosdep docs](https://docs.ros.org/en/independent/api/rosdep/html/contributing_rules.html), but I cant be bothered)

Firstly you need to install `pip` (or `pip3` if you are using python3) with
```sh
sudo apt install python-pip
```
or
```sh
sudo apt install python3-pip
```

Next, install neccesary python packages (replace `pip` with `pip3` if you are using python3):
```sh
pip install canlib
```

You also need to follow instructions on installing [Kvaser LINUX Driver and SDK](/vendor-drivers-tools/kvaser-linuxcan).

# Nodes

