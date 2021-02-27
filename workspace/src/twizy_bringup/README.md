# twizy_bringup

This package is used to house launch and config files used to start the entire Twizy system or it's various subsystems.
This package should depend on all other packages in the workspace.

# Launch Files

## dual_gnss

Publishes pose data from the Twizy's dual RTK-GNSS recievers

### Arguments

`host_l` (`str`, default: "192.168.0.222")

&emsp;Host to recieve data from for left GNSS reciever

`host_l` (`str`, default: "192.168.0.223")

&emsp;Host to recieve data from for right GNSS reciever

`port_l` (`int`, default: 55555)

&emsp;Port to recieve data on for left GNSS reciever

`port_r` (`int`, default: 55555)

&emsp;Port to recieve data on for right GNSS reciever

### Starting

```sh
roslaunch twizy_bringup dual_gnss
```