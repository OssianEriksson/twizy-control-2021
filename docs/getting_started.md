# Getting Started <!-- omit in toc -->

This page is intended to help you get an overview of what is contained in this repository.

# Table of Contents <!-- omit in toc -->

- [About](#about)
  - [Background](#background)
  - [Design Goals](#design-goals)
- [Installation](#installation)
  - [Prerequisites](#prerequisites)
    - [Git](#git)
  - [Cloning the Repo](#cloning-the-repo)
    - [Using SSH](#using-ssh)
    - [Using HTTPS](#using-https)
- [Taking a Look Around the Repo](#taking-a-look-around-the-repo)

# About

## Background

In preparation of the spring of 2018 Chalmers University of Techology bought a Renault Twizy, origininally a small two seat electric car.
This car was then modified as part of multiple bachelor's thesis projects during the spring of that year.
The back seat was removed to house a computer running the Ubuntu operating system, control electronics was installed to enable computer control of the Twizy and multiple sensors were mounted on the car.
The first self-driving implementations made use of [Apollo Auto](https://apollo.auto) which is based on [ROS](https://www.ros.org/).
Apollo was abandoned between 2019 and 2020 because of poor documentation in favor of a custom implementation in ROS.

The code in this repository has its roots in code from 2019, but the project has been completely rewritten and restructured to aid further development on the car.
This repo was created in the spring of 2021 as part of a Bachelor's thesis project on automated path planning and object avoidance.
Code in this repository is as of now only ROS-based.

## Design Goals

In restructuring the code from 2019 and 2020 the following goals were kept in mind as part of the bachelor's project of that year:

1. To update the code structure to be in line with ROS recommendations and practises seen in other code bases
2. To unify the codebase into a single practice for writing code
3. To document the project as a first priority and everything else related to the twizy as a second priority in these files that you are reading now

These things were done to aid in further development of the Twizy's ROS software stack by
1. Making existing code and harware knowledge easier to grasp, and
2. Setting a barreier of entry to writing code so the repo can remain well structured in the future as new projects are carried out on the Twizy using ROS.
   
After 2021 (the year of writing) this repo will hopefully be handed over to next years students and no longer be moderated by it's original creators.

# Installation

The first thing you should do to run some code or make some edits is to copy all the files in this repo to your local computer, but first there are some other things you might need:

## Prerequisites

- [Ubuntu](https://ubuntu.com/) – Ubuntu 18 is the current operating system on the Twizy. If your are using Windows it is recomended to dual boot Ubuntu.
- [ROS](https://www.ros.org/) – We're currently using ROS 1 (ROS Melodic Morenia) which is the current ROS version on the Twizy.
- [git](https://git-scm.com/) – Version control system. Your inside a git repository (repo) right now! On can be installed on debian based systems with `sudo apt install git`.

### Git

If you havn't noticed, this repository uses [git](https://git-scm.com/).
If you don't know what git is, I suggest you to read up on it before going further.
You can still explore the repo here on github without knowing what git is, but even in order to run code, not even editing it, some tiny knowledge of git is neccessary.

## Cloning the Repo

I personaly recommend using SSH to clone the repo.
It might take a little longer to set up, but it saves time in the long run.

### Using SSH

Follow the instructions on [using ssh keys with Github](https://docs.github.com/en/github/authenticating-to-github/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent).

Finally you can clone the repo:
```sh
git clone https://github.com/OssianEriksson/twizy-control-2021.git
```

### Using HTTPS

Clone the repo:
```sh
git clone https://github.com/OssianEriksson/twizy-control-2021.git
```

# Taking a Look Around the Repo

Below is a list of some directories of interest.
You can browse the repo on [Github](https://github.com/OssianEriksson/twizy-control-2021) or on your local file system if you have first [cloned the repo](#cloning-the-repo).

- Documentation can be found in [doc](/doc)
- ROS-packages can be found in the [src](/workspace/src) sub-directory of the [catkin workspace](/workspace). This includes all of the production code but also packages used for testing production code, e.g. [2020's parallel parking](/workspace/src/twizy_parking_2020). 
- Radom, personal, poorly documented experiments for learning the tools used in projects here can be found in [playground](/playground).
