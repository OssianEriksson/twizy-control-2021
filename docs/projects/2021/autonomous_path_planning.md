# Autonomous Renault Twizy - Autonomous Path Planning

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
roslaunch twizy_control_2021 webots.launch
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

# Authors

- Jonathan Almgren
- Arvid Enliden
- Ask Uv
- Ossian Eriksson
- Albin Ekelund Carlsson
- Emil Hölvold