# twizy_localization

ROS-package for interpreting and tranfsforming the Twizy's GNSS data.

# Installation

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
rosdep install twizy_localization
```

# ROS API

## gnss_pose

This node reads GNSS positioning data from two or more GNSS devices and publishes position and orientation data.
In order to determine position correctly at least three GNSS data sources are required to eleminate all degrees of freedom.
However running only two GNSS devices is supported with the result that rotation around the axis between the GNSS recievers will always be reported as zero.
This package uses a lot of made up or guessed covariance math, so have a look over it if you feel you need to sadden yourself for some reason.

This package uses tf2 to read positions of GNSS antennas in local reference frame (relative to the robot).
It looks for the transform that transforms this local data to the global reference frame in which the GNSS devices report their location.

### Published Topics

`/gnss/pose` ([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))

&emsp;Estimate of the Twizy's current pose

### Subscribed Topics
`/gnss/fix` ([sensor_msgs/NavSatFix](https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html))

&emsp;Topic to read GNSS data from. The messages contain a header with a `frame_id` field which is used to link that message to a specific link in the tf tree

### Parameters

`~center_link` (`str`, required)

&emsp;Name of tf frame which is supposed to be located at the centeroid of all currently active GNSS recievers. The published pose will contain the position and orientation of this link in the frame provided by `~map_frame`

`~map_frame` (`str`, default: "map")

&emsp;`frame_id` of published messages on `/gnss/fix`

`~frequency` (`float`, default: 30.0)

&emsp;Frequency to publish messages at (Hz)

`~timeout` (`float`, default: 0.5)

&emsp;If sensor data is older than `~timeout` (s) it will not be included in computation. Since this node is likely to be used to publish transforms, it will not always be possible to easily predict correct positions of all GNSS recievers at the time of update (as controlled by `~frequency`). Therefore when fusing GNSS data into a single pose, data from different time frames are combined. If a sensor for example stops publishing data, without the `~timeout` setting its position in the global frame when the last message was sent will be remembered by this node, which will then give catastrophically wrong output data. `~timeout` should therefore be kept short (but not shorter than the rate the GNSS devices publish messages at), and `~frequency` should generally be kept high

`~differential` (`bool`, default: false)

&emsp;If this parameter is true, the first position measurement will be subtracted from all further pose estimates so that the pose starts at the origin of the coordinate system. Orientation is unaffected

### Starting

Start the nodes with
```sh
rosrun twizy_localization gnss_pose
```
