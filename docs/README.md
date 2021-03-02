# twizy-control-2021 Documentation <!-- omit in toc -->

These pages contain information regarding ROS related development on the autonomous Renault Twizy at Chalmers University of Technology, Gothenburg.
This includes documentation of software as well as hardware.

# Table of Contents <!-- omit in toc -->

- [Getting Started](#getting-started)
- [Usage Instructions](#usage-instructions)
- [Hardware](#hardware)
- [System Details](#system-details)
- [Development Guidelines and Norms](#development-guidelines-and-norms)

# Getting Started

To get to know the projects and this repo you should start by looking at

- [Getting Started](getting_started.md) – introduction to the projects

# Usage Instructions

For information on usage you can read

- [Usage Instructions](usage.md) – information on how to run the various systems and subsystems

# Hardware

For information on harware, practical instructions and similar, see

- [Hardware](hardware) – practical information related to the Twizy

# System Details

Information on how the different parts of the machinery fits together – for package specific information please see the packages individual readme:s located in the corresponding child directory of [/workspace/src](/workspace/src).
We also try to keep the source code short and densly commented, so if you are interested in package implementation details, the source code can be your friend as a last resort.

For information on simulation of the Twizy for testing and evaulation purposes see

- [Simulation Using Webots](webots.md) – Real time simulation using [webots](https://cyberbotics.com/)

For information on general purpose subsystems, please see

- [Subsystem Details](subsystems) – List of documented subsystems

For complete project documentations, e.g. complete systems put together as bachelor's thesis projects or other complete systems, see

- [Project Details](projects) – List of documented projects

# Development Guidelines and Norms

Before you develop your own code in this repo you should have some idea of the guidlines and structure of the code.
You expect to have a reasonale knowledge of ROS before diving into developing your own code, but this repo is also deliberately very stubbornly structured to encorage you to create nicely formatted code and file structures.
In the long run having a uniform coding style thougout the project helps new people get the hang of the project quicker and reduces the number of design desicions the developers need to make.
Please see

- [Development Guidlines and Norms](development_guidlines.md) – What to do, what not to do and suggested reading