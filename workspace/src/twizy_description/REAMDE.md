# twizy_description

This package contains files related to Twizy parameters and configuration.
It is a pure utility package for other packages to depend on and should not contain any code.

# Directories

## config

This directory is for YAML config files which for example can be used as ROS parameters.

## protos

This directory contains models of robots and objects which are compatible with the webots simulation program.

Files in this directory may need to be pre-processed by some scripting language, for example python.

## rviz

This directory contains rviz config files which are used to start rviz with already open views and visualisation tool (so you dont have to reconfigure rviz every time you start it).
These config files can be opend directly from the UI in rviz, but they are probably also associated with a launch file which can be used to start rviz instead.

## urdf

This directory contains models of robots and objects which are compatible with rviz, gazebo and tf.

Files in this directory may need to be pre-processed by some scripting language, for example xacro.