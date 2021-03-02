# Getting Started <!-- omit in toc -->

# Table of Contents <!-- omit in toc -->

- [About](#about)
- [Installation](#installation)
  - [Prerequisites](#prerequisites)
  - [Cloning the Repo](#cloning-the-repo)
- [Directory Structure](#directory-structure)

# About

# Installation

How to get a local copy of this project on your computer.

## Prerequisites

- [Ubuntu](https://ubuntu.com/) – Ubuntu 18 is the current operating system on the Twizy. If your are using Windows it is recomended to dual boot Ubuntu.
- [ROS](https://www.ros.org/) – We're currently using ROS 1 (ROS Melodic Morenia) which is the current ROS version on the Twizy.
- [git](https://git-scm.com/) – Version control system. Your inside a git repository (repo) right now! On can be installed on debian based systems with `sudo apt install git`.

## Cloning the Repo

Clone the repo:
```sh
git clone https://github.com/OssianEriksson/twizy-control-2021.git
```

# Directory Structure

- Documentation can be found in [doc](/doc)
- ROS-packages can be found in the [src](/workspace/src) sub-directory of the [catkin workspace](/workspace). This includes most of our production code but also packages used for testing, e.g. [2020's parallel parking](/workspace/src/twizy_parking_2020). 
- Radom, personal, poorly documented experiments for learning the tools we are using for this project can be found in [playground](/playground).

