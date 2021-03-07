# neural_proc

Nodes for neural network processing of data

# ROS API

## yolo

Classifies and locates objects in images.

### Subscribed Topics

`image` ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))

&emsp;Image to process

### Published Topics

`bounding_boxes` ([twizy_msgs/BoundingBoxes](../twizy_msgs/msg/BoundingBoxes.msg))

&emsp;Bounding boxes of objects in detection image

### Starting

To use the SBP protocol over a TCP socket as a GNSS data source, start the node using
```
rosrun piksi_multi_rtk piksi
```
