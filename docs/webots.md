# Simulation Using Webots <!-- omit in toc -->

[Webots](https://cyberbotics.com/) is a simulation software for robotics.

# Table of Contents <!-- omit in toc -->

- [Introduction](#introduction)
- [ROS Controller](#ros-controller)
  - [Published Topics](#published-topics)
- [Interfacing With the Webots ROS Controller](#interfacing-with-the-webots-ros-controller)
  - [Control Node](#control-node)
- [Launch Files](#launch-files)
  - [webots](#webots)
  - [webots_keyboard](#webots_keyboard)
- [System Details](#system-details)
  - [Launch File](#launch-file)
    - [The Code Explained](#the-code-explained)

# Introduction

Before webots was integrated, surveys were done on gazebo (which comes prepackaged with ROS) and webots.
Webots was recommended by Rickard Karlsson who at that time was responsible for the CASE lab at chalmers, who said webots was being used in multiple master thesis projects.
gazebo was found to have worse performance (20-30% of realtime compared to 90-100% of realtime for our test scenarios) and less stable simulation (the video game classic phenomenon of objects spazzing out when the simulation is stressed by for example driving the robot into an obstacle) out of the box.
Both gazebo and webots by default use the same ODE physics engine, so with some tweaking is likely that simulations in gazebo could run much nicer than what was observed, but we did not want to spend time reading up on and tweaking parameters.

Webots does have some drawbacks compared to gazebo:
- It doesn't come bundeled with ROS
- It uses its own custom file format, different from what [tf](https://docs.ros.org/en/foxy/Tutorials/tf2.html) uses which is URDF (which is supported by gazebo)
- Less focus on ROS control, and the built in ROS controller in webots is very sparse

Webots can simulate sensors such as cameras and GNSS recievers.

Files reating to the core webots simulation can be found in the [twizy_webots](/workspace/src/twizy_webots) package.

# ROS Controller

To control the simulation the built in ROS controller of webots is used instead of a custom one.
This is partially for performance reasons, so conversion between image formats (webots image data to ROS Image message) doesn't have to be performed in python.
Although this does have some drawbacks as the default ROS controller is quite limited.
For example the [publish_camera_info](/workspace/src/twizy_webots#twizy_webots/nodes/publish_camera_info) is required to publish camera info for simulated cameras, which isn't handeled by the webots ROS controller.
The built in ROS controller also for example doesn't support GNSS data covariance and noise.

There is quite decent documentation provided by cyberbotics for webots, protos, world files and pleny of examples provided for using the provided ROS controller.
Sometimes though it can be neccessary to look at the source of the provided webots ROS controller which can be found [here](https://github.com/cyberbotics/webots/tree/master/projects/default/controllers/ros).

## Published Topics

This controller publishes the following topics:

`/front/camera/depth/image_rect_raw` ([sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html))

&emsp;Depth image from simulated Intel RealSense D435 camera

`/front/camera/aligned_depth_to_color/image_raw` ([sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html))

&emsp;Color image from simulated Intel RealSense D435 camera with same FOV and dimensions as depth image published on `/front/camera/depth/image_rect_raw`

`/gnss/{left|right}/navsatfix_best_fix` ([sensor_msgs/NavSatFix](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/NavSatFix.html))

&emsp;GNSS data from left and right simulated GNSS reciever on two separate topics. Covariance will always be zero (unknown) as discussed above

# Interfacing With the Webots ROS Controller

Instead of letting the rest of the system to be simulated communicate directly with the webots ROS controllers, many different nodes are used to form a layer in between the webots ROS controller and the rest of the ROS graph.
The nodes typically let the user remap their output topics which isn't possible in any nice way that I know of (I have hacked it in the past using `controllerArgs`) and change the `frame_id` of messages.
These nodes take care of one type of device each, for example there is

- [camera](/workspace/src/twizy_webots#camera) to republish images on better topic names and also publish camera info, which webots doesn't do by default
- [gps](/workspace/src/twizy_webots#gps) to republish GPS (GNSS) data and simulate noise.
  **Note that webots has built in support for GPS noise which we dont use** since there is no way to access the standard deviations used through the webots ROS controller. Instead this node adds its own ("known") noice.

## Control Node

The most important node however is the [control](/workspace/src/twizy_webots#control) node.
Other nodes mentioned previously which interface with the webots ROS controller have been designed to be general and work for any robot.
This node is instead specific to the Twizy, and uses parameters which you are strongly recommended to set from [twizy_description/config/twizy_properties.yaml](/workspace/src/twizy_description/config/twizy_properties.yaml).
This node mainly takes care of sending driving control messages to the simulation.

# Launch Files

## webots

Start the webots simulation of the Twizy.

### Nodes <!-- omit in toc -->

[control](/workspace/src/twizy_webots#control)

[camera](/workspace/src/twizy_webots#camera)

&emsp;One instance started for front RealSense camera (color image)

[range_finder](/workspace/src/twizy_webots#range_finder)

&emsp;One instance started for front RealSense camera (depth image)

[gps](/workspace/src/twizy_webots#gps)

&emsp;One instance started for the Twizy's left GNSS reciever and one for the right

### Arguments <!-- omit in toc -->

`world` (`str`, default: "$(find twizy_webots)/worlds/flat.wbt")

&emsp;Path to the world to load

`mode` (`str`, default: "realtime")

&emsp;Webots startup mode

`no-gui` (`bool`, default: false)

&emsp;Start Webots with minimal GUI

### Starting <!-- omit in toc -->

```sh
roslaunch twizy_bringup webots.launch
```

## webots_keyboard

Start the webots simulation with a keyboard controller to drive the Twizy around.

### Included Launch Files <!-- omit in toc -->

[webots.launch](#webots)

### Nodes <!-- omit in toc -->

[local_keyboard](/workspace/src/twizy_control#local_keyboard)

### Arguments <!-- omit in toc -->

`world` (`str`, default: "$(find twizy_webots)/worlds/flat.wbt")

&emsp;Path to the world to load

`mode` (`str`, default: "realtime")

&emsp;Webots startup mode

`no-gui` (`bool`, default: false)

&emsp;Start Webots with minimal GUI

### Starting <!-- omit in toc -->

```sh
roslaunch twizy_bringup webwebots_keyboardots.launch
```

# System Details

This section describes what happens when you start the [webots.launch](#webots) launch file, which is the main launch file for webots.

## Launch File

The contents of the launch file is as follows:
```xml
<?xml version="1.0"?>
<launch>
    <arg name="world" default="$(find twizy_webots)/worlds/flat.wbt" doc="Path to the world to load" />
    <arg name="mode" default="realtime" doc="Startup mode" />
    <arg name="no-gui" default="false" doc="Start Webots with minimal GUI" />

    <rosparam ns="twizy_properties" command="load" file="$(find twizy_description)/config/twizy_properties.yaml" />

    <param name="/use_sim_time" type="bool" value="true" />

    <node name="twizy_webots" pkg="webots_ros" type="webots_launcher.py" args="--world=$(arg world) --mode=$(arg mode) --no-gui=$(arg no-gui)" launch-prefix="bash -c '$(find twizy_webots)/scripts/proto; $0 $@' " required="true" />

    <node name="control" pkg="twizy_webots" type="control" required="true" />

    <remap from="/piksi_multi_rtk/position_covariance" to="/twizy_properties/gnss/position_covariance" />
    <param name="piksi_multi_rtk/position_covariance_type" type="int" value="3" />
    <rosparam param="/gps_reference">[57.671667, 11.980833, 0]</rosparam>
    <node ns="piksi_multi_rtk" name="left_gnss" pkg="twizy_webots" type="gps" required="true">
        <remap from="/navsatfix" to="/gnss/left/navsatfix_best_fix"/>

        <param name="device" value="left_gnss" />
    </node>
    <node ns="piksi_multi_rtk" name="right_gnss" pkg="twizy_webots" type="gps" required="true">
        <remap from="/navsatfix" to="/gnss/right/navsatfix_best_fix"/>

        <param name="device" value="right_gnss" />
    </node>

    <remap from="/realsense_d435/aligned_depth_to_color/fov" to="/twizy_properties/realsense_d435/depth/fov" />
    <node ns="realsense_d435/aligned_depth_to_color" name="front_realsense_range_finder" pkg="twizy_webots" type="range_finder" required="true">
        <remap from="/image_raw" to="/front/camera/depth/image_rect_raw"/>
        <remap from="/camera_info" to="/front/camera/depth/camera_info"/>

        <param name="device" value="front_realsense_depth_camera" />
        <param name="frame_id" value="front_camera" />
    </node>
    <node ns="realsense_d435/aligned_depth_to_color" name="front_realsense_depth_aliged_color" pkg="twizy_webots" type="camera" required="true">
        <remap from="/image_raw" to="/front/camera/aligned_depth_to_color/image_raw"/>
        <remap from="/camera_info" to="/front/camera/aligned_depth_to_color/camera_info"/>

        <param name="device" value="front_realsense_aligned_depth_to_color_camera" />
        <param name="frame_id" value="front_camera" />
    </node>
</launch>
```

### The Code Explained

The line
```xml
<rosparam ns="twizy_properties" command="load" file="$(find twizy_description)/config/twizy_properties.yaml" />
```
loads parameters about the twizys physical dimensions and sensors into the ROS parameter server.
This data is mainly used by the [control](/workspace/src/twizy_webots#control) node.

The line
```xml
<param name="/use_sim_time" type="bool" value="true" />
```
is used to link ROS' internal clock with the time published on the `/clock` topic.
Later we will tell webots to publish its simulation time to this topic.
If you for example wind back your webots simulation and errors appear telling you that ROS time moved backwards, this is why.
Also remember that parameters are stored on the parameter server until the ROS master is closed (I think).
So if wired things are happening with the ROS time at times (like the time always beeing 0), make sure the `/use_sim_time` is set correctly.

The line
```xml
<node name="twizy_webots" pkg="webots_ros" type="webots_launcher.py" args="--world=$(arg world) --mode=$(arg mode) --no-gui=$(arg no-gui)" launch-prefix="bash -c '$(find twizy_webots)/scripts/proto; $0 $@' " required="true" />
```
does mainly two things.
It starts the webots program for one thing, using the webots_ros package which you need to install.
But before starting the node we must make sure models of the robot we want to simulate is available to webots.

As discussed above, one major drawback of webots is that it isn't compatible with URDF files, but istead relies on proto files.
URDF files are still needed though for other parts of the ROS stack.
To partially avoid having to adjust parameters in two separate models of the same robot whenever we would like to change something in the robot, URDF and proto files are generated from a YAML config file before passing the robot models on to whichever programs need them.
It is common to store robot model related files and config files in a robot_description package, in this case [twizy_description](/workspace/src/twizy_description).
Without setting additional settings in webots however, the simulation program is only able to read model files from [twizy_webots/protos](/workspace/src/twizy_webots/protos).
So we want to generate protos from config files and place the output in twizy_webots/protos.

Webots does natively support lua scripting in proto files, but to read YAML config files from lua would require a lot of configuration and installing of extra dependicies.
Instead we use python for this and process the files before passing them to webots.
By for example running the script [Twizy.proto.py](/workspace/src/twizy_description/protos/Twizy.proto.py) a finished proto file is printed to standard output.

The script [twizy_webots/scripts/proto](/workspace/src/twizy_webots/scripts/proto) handles running the proto templates of [twizy_description/protos](/workspace/src/twizy_description/protos) and placing them in twizy_webots/protos.
To make sure the script runs before the webots simulation program is started we use the trick with the `launch-prefix` attribute in webots.launch.

The robot file [Twizy.proto.py](/workspace/src/twizy_description/protos/Twizy.proto.py) contains some important settings which are needed for the simulation to work as described here.
Firstly the controllerArgs `--clock` and `--use-sim-time` are specified in order to publish the `/clock` topic for ROS time.
Secondly some controllerArgs are provided which are not intended for the webots ROS controller itself, but for ROS.
Webots' ROS controller will complain about these because it is not programmed to handle them, but they still have the intended effect even though they generate errors in the webots console which you can safetly ignore.
They are used to set parameters or to remap topics, which as far as I know isn't natively supported by the webots ROS controller, another drawback of using it.
If you find a better way to do this, please update the code.
So if you didn't pick it up, the publishing of sensor message topics is largely handeled by the webots ROS controller and not by any custom node.

While we are at it, another important thing to notice for the simulation to work as described here is to set the `gpsCoordinateSystem "local"` in the WorldInfo node of your webots world.
See [gps](/workspace/src/twizy_webots#gps) for more info.

The line
```xml
<node name="control" pkg="twizy_webots" type="control" required="true" />
```
just starts the [control](/workspace/src/twizy_webots#control) node, read more about it in the package documentation.

The lines past this are used to start nodes which reads from topics published by the ROS controller, possibly transforms or augments the data a bit and then republishes the messages on other topics which can then be remapped.
These nodes usually require some parameters which are also sometimes fetched from [twizy_description/config/twizy_properties.yaml](/workspace/src/twizy_description/config/twizy_properties.yaml).
That way the parameters set for the ROS nodes are sure to match the values present in the model files of the Twizy, since these files are generated from that very same YAML file as described above.

For example the line
```xml
<remap from="/realsense_d435/aligned_depth_to_color/fov" to="/twizy_properties/realsense_d435/depth/fov" />
```
sets the `fov` parameter for the [camera](/workspace/src/twizy_webots#camera) and [range_finder](/workspace/src/twizy_webots#range_finder) nodes started further down:
```xml
<node ns="realsense_d435/aligned_depth_to_color" name="front_realsense_depth_aliged_color" pkg="twizy_webots" type="camera" required="true">
    ...
```
and
```xml
<node ns="realsense_d435/aligned_depth_to_color" name="front_realsense_range_finder" pkg="twizy_webots" type="range_finder" required="true">
    ...
```