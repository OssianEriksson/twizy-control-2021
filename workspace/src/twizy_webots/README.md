# twizy_webots

ROS-package for simulating the Twizy in the webots simulation program.
This package makes use of the [built in webots ROS controller](https://cyberbotics.com/doc/guide/using-ros#standard-ros-controller).

# Installation

Install [webots_ros](http://wiki.ros.org/webots_ros#Installation) and it's dependicies.

# ROS API

## controller

This is a node for interfacing with the webots simulation.
It mainly reads [twizy_msgs/CarControl](../twizy_msgs/msg/CarControl.msg) messages and sends the contained reference values to the simulation.

### Subscribed Topics

`twizy_control` ([twizy_msgs/CarControl](../twizy_msgs/msg/CarControl.msg))

&emsp;Reference control (steering angle and speed) values for the Twizy to pass on to the webots simulation

### Parameters

`~realsense_fps` (`float`, default: 30.0)

&emsp;Realsense cameras' number of frames per second

`~gnss_ups` (`float`, default: 30.0)

&emsp;GNSS recievers' number of updates per second

### Starting

Start the node with
```sh
rosrun twizy_webots controller
```

## publish_camera_info

Publishes a `[/...]/camera_info` topic corresponding to a topic containing [sensor_msgs/Image](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html) data.
The camera is assumed to have no distorsion.

### Subscribed Topics

`[~image_topic]` ([sensor_msgs/Image](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html))

&emsp;Topic assumed to contain undistorted image data to publish camera info for. Topic name is controlled by the `~image_topic` [parameter](#Parameters)


### Published Topics

`[/...]/camera_info` ([sensor_msgs/CameraInfo](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html))

&emsp;Camera info messages corresponding to images recieved on the `[~image_topic]` [topic](#Subscribed-Topics). See the `~image_topic` [parameter](#Parameters), which controls the name of this topic.

### Parameters

`~fov` (`float`)

&emsp;Horizontal field of view of camera (radians)

`~image_topic` (`str`)

&emsp;Topic of [sensor_msgs/Image](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html) to publish camera info for. The output topic name gets chosen based on this parameter, so for example in case `~image_topic` is set to "/camera/test/image_raw" this node will publish "/camera/test/camera_info"

### Starting

```sh
rosrun twizy_webots publish_camera_info _fps:=<fps> _image_topic:=<image_topic>
```
