# twizy_webots

ROS-package for simulating the Twizy in the webots simulation program.
This package makes use of the [built in webots ROS controller](https://cyberbotics.com/doc/guide/using-ros#standard-ros-controller).

# Installation

Install [webots_ros](http://wiki.ros.org/webots_ros#Installation) and it's dependicies.

# ROS API

## control

This is a node for interfacing with the webots simulation.
It mainly reads [twizy_msgs/CarControl](../twizy_msgs/msg/CarControl.msg) messages and sends the contained reference values to the simulation.
Note that parameters mirror what can be found in [twizy_msgs/config/twizy_properties.yaml](/workspace/src/twizy_description/config/twizy_properties.yaml).

### Subscribed Topics

`twizy_control` ([twizy_msgs/CarControl](../twizy_msgs/msg/CarControl.msg))

&emsp;Reference control (steering angle and speed) values for the Twizy to pass on to the webots simulation

### Parameters

`/twizy_properties/wheel/radius` (`double`, required)

&emsp;Wheel radius of the Twizy (m)

`/twizy_properties/wheel/max_speed` (`double`, required)

&emsp;Max speed of the Twizy (m/s)

`/twizy_properties/wheel/max_steering_angle` (`double`, required)

&emsp;Maximum steering angle of the Twizy (rad)

`/twizy_properties/chassis/wheelbase` (`double`, required)

&emsp;Wheelbase of the Twizy (m), (used for Ackermann calculations)

`/twizy_properties/chassis/track` (`double`, required)

&emsp;Track of the Twizy (m), (used for Ackermann calculations)

### Starting

Start the node with
```sh
rosrun twizy_webots control
```

## camera

Node for interfacing with the webot ROS controller.
It publishes images from a webots [Camera](https://cyberbotics.com/doc/reference/camera) node.

### Subscribed Topics

`/<model_name>/<device>/image` ([sensor_msgs/Image](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html))

&emsp;Images published by the webots ROS controller. `model_name` and `device` are parameters

### Published Topics:

`/image_raw` ([sensor_msgs/Image](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html))

&emsp;Images from `/<model_name>/<device>/image` are just republished on this topic with an updated `header.frame_id` field. This topic is ment to be remapped

`/camera_info` ([sensor_msgs/CameraInfo](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html))

&emsp;Camera info corresponding to `/image_raw`. This topic is ment to be remapped

### Parameters

`/model_name` (`str`, default value is read from the `/model_name` topic, published by webots ROS controller)

&emsp;Name of the model in the webots simulation

`~device` (`str`, required)

&emsp;Name of device in webots proto

`~frame_id` (`str`, default: Trunkated version of frame_id set by webots ROS controller)

&emsp;frame_id to republish images under

`fps` (`float`, default: 30.0)

&emsp;Camera frames per second

`fov` (`float`, default: pi / 2)

&emsp;Horizontal field of view

`distorsion_model` (`str`, default: "plumb_bob")

&emsp;Used when publishing `/camera_info`. See [sensor_msgs/CameraInfo](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html) for definition

`D` (`yaml`, default: \[0, 0, 0, 0, 0\])

&emsp;Used when publishing `/camera_info`. See [sensor_msgs/CameraInfo](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html) for definition

`R` (`yaml`, default: \[1, 0, 0, 0, 1, 0, 0, 0, 1\])

&emsp;Used when publishing `/camera_info`. See [sensor_msgs/CameraInfo](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html) for definition

### Starting

Start the node with
```sh
rosrun twizy_webots camera
```

## range_funder

Node for interfacing with the webot ROS controller.
It publishes images from a webots [RangeFinder](https://www.cyberbotics.com/doc/reference/rangefinder) node.

### Subscribed Topics

`/<model_name>/<device>/range_image` ([sensor_msgs/Image](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html))

&emsp;Depth images published by the webots ROS controller. `model_name` and `device` are parameters

### Published Topics:

`/image_raw` ([sensor_msgs/Image](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html))

&emsp;Depth images from `/<model_name>/<device>/range_image` are just republished on this topic with an updated `header.frame_id` field. This topic is ment to be remapped

`/camera_info` ([sensor_msgs/CameraInfo](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html))

&emsp;Camera info corresponding to `/image_raw`. This topic is ment to be remapped

### Parameters

`/model_name` (`str`, default value is read from the `/model_name` topic, published by webots ROS controller)

&emsp;Name of the model in the webots simulation

`~device` (`str`, required)

&emsp;Name of device in webots proto

`~frame_id` (`str`, default: Trunkated version of frame_id set by webots ROS controller)

&emsp;frame_id to republish images under

`fps` (`float`, default: 30.0)

&emsp;Camera frames per second

`fov` (`float`, default: pi / 2)

&emsp;Horizontal field of view

`distorsion_model` (`str`, default: "plumb_bob")

&emsp;Used when publishing `/camera_info`. See [sensor_msgs/CameraInfo](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html) for definition

`D` (`yaml`, default: \[0, 0, 0, 0, 0\])

&emsp;Used when publishing `/camera_info`. See [sensor_msgs/CameraInfo](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html) for definition

`R` (`yaml`, default: \[1, 0, 0, 0, 1, 0, 0, 0, 1\])

&emsp;Used when publishing `/camera_info`. See [sensor_msgs/CameraInfo](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html) for definition

### Starting

Start the node with
```sh
rosrun twizy_webots range_finder
```

## gps

Node for interfacing with the webot ROS controller.
It publishes images from a webots [GPS](https://www.cyberbotics.com/doc/reference/gps) node.
It is important that the `gpsCoordinateSystem` field in the [WorldInfo](https://cyberbotics.com/doc/reference/worldinfo) node is set to "local" for the webots ROS controller to publish the correct topics.

### Subscribed Topics

`/<model_name>/<device>/values` ([geometry_msgs/PointStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PointStamped.html))

&emsp;Simulated GPS coordinates. `model_name` and `device` are parameters. It is important that the `gpsCoordinateSystem` field in the [WorldInfo](https://cyberbotics.com/doc/reference/worldinfo) node is set to "local" for the webots ROS controller to publish this topic.

### Published Topics:

`/navsatfix` ([sensor_msgs/NavSatFix](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/NavSatFix.html))

&emsp;GPS positioning data. The values from the `/<model_name>/<device>/range_image` are transformed and then republished here. This topic is ment to be remapped

### Parameters

`/model_name` (`str`, default value is read from the `/model_name` topic, published by webots ROS controller)

&emsp;Name of the model in the webots simulation

`~device` (`str`, required)

&emsp;Name of device in webots proto

`~frame_id` (`str`, default: Trunkated version of frame_id set by webots ROS controller)

&emsp;frame_id to republish images under

`ups` (`float`, default: 30.0)

&emsp;GPS updates per second

`status` (`int`, default: 0)

&emsp;Used whe publishing `/navsatfix`. see [sensor_msgs/NavSatFix](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/NavSatFix.html) for definition

`service` (`int`, default: 8)

&emsp;Used whe publishing `/navsatfix`. see [sensor_msgs/NavSatFix](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/NavSatFix.html) for definition

`position_covariance` (`yaml`, default: [0, 0, 0, 0, 0, 0, 0, 0, 0])

&emsp;Covariance matrix for noise to add to the GPS data recieved from the webots ros controller before republising the data on `/navsatfix`. It is a row major matrix with units of (m^2). The matrix is specified in a east-north-up (ENU) coordinate system. See also [sensor_msgs/NavSatFix](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/NavSatFix.html)

`position_covariance_type` (`int`, default: 0)

&emsp;Used whe publishing `/navsatfix`. see [sensor_msgs/NavSatFix](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/NavSatFix.html) for definition

`/gps_reference` (`yaml`, default: [0, 0, 0])

&emsp;Vector containing initial latitude (degrees), longitude (degrees) and height above WGS84 ellipsoid (m) of the models initial position.

### Starting

Start the node with
```sh
rosrun twizy_webots gps
```