# Development Guidlines and Norms <!-- omit in toc -->

This page will help you get started writing code.
Although it might seem tempting to just jump in and get started writing, you are encoraged to learn a bit about coding practices beforehand.
If you however already have decent experience in ROS, a lot of what is said here might not be news to you, and you might already be ready to "jump in".
When the creators of this repository started out, we had never heard of ROS before and this repo only took real shape about half way through the project after atleast I had gathered around 100-150 hours of experience in ROS.
So two two three things to keep in mind:

1. This repository was created by beginners as their first or second ever project using ROS, everything will not be perfect or the way you expect with thousands of hours of experience
2. We still had spent a quite a lot of time learning about ROS for a bachelor's thesis before the code here was set up
3. By structuring this repository so rigorously we intend to accelerate learning about ROS in future projects by right away enforcing good coding practices and so newcomers can learn quickly what to do by looking at what is already here

# Table of Contents <!-- omit in toc -->

- [Prerequisites](#prerequisites)
- [Git and Repository Structure](#git-and-repository-structure)
  - [Branches](#branches)
  - [Playground](#playground)
- [Writing Documentation](#writing-documentation)
- [Writing Tests](#writing-tests)
- [Licence](#licence)
- [ROS Practices](#ros-practices)
  - [Recommended Reading](#recommended-reading)
- [Writing Code (Finally...)](#writing-code-finally)
  - [Python](#python)
    - [Code conventions](#code-conventions)
  - [Visual Studio Code](#visual-studio-code)
  - [Are We There Yet?](#are-we-there-yet)

# Prerequisites

Before going further, you should atleast read throgh and understand what is presented under [Getting Started](README.md#getting-started).
This includes some knowledge of git and ROS in general.

Even if some practice is not documented in this file, I suggest when starting a new project to start by copying and old project.
Preferably you maybe even recreate it command by command and file by file to get a good feeling for the purpose of every file.
Google is your friend when you don't understand what a perticular file or line does.

# Git and Repository Structure

## Branches

The used in this project are:

- **master** - The stable and only longrunning branch. Code should have been confiremed to work before it is merged into master.
- **dev-\<name\>**, where \<name\> is a name of a feature to be implemented or a bug to be fixed - Create a new dev- branch when you need to change something in the code. After testing the updated code, this branch can get merged into master. The original dev- branch can then be deleted.
- **playground-\<name\>**, where \<name\> is your username or a description - Code in a playground- branch should be completely separate from production code. Playground code should be placed in the [playground](/playground) directory. playground- branches can get merged into master and then deleted once development on the branch is finished.

When creating a branch, name it either dev-\<name\> or playground-\<name\> according to the above:
```sh
git checkout -b dev-my-amazing-new-feature
```

When writing code, **NEVER DO IT DIRECTLY IN MASTER**.
You should always create a new branch with one of the prefixes described above.
This is standard git practice.
Larger and slower moving projects (where entire systems are not developed in months) usually have a lot more different branch types.
[This](https://nvie.com/posts/a-successful-git-branching-model/) is a popular model for example.
As of now we keep to this simple structure to not have to worry too much about what kinds of things can go in what branches.

## Playground

The [playground](playground) is for trying out and as the name suggest playing with code.
Code in playground should be kept completely separate from production code.
This directory is associated with playground-<name> branches, see [Branches](#branches).
You are encoraged to share smaller and separate project with others in your project group using playground if you dont have a separate repository for this.

# Writing Documentation

The goal is for all code in this repository to be very well documented.
Documenting too early can be dangerous if you then later restructure a big part of the project, but try to keep up with documentation atleast of the stable parts of your system.

Different kinds of documentation goes in different places.
Broader system and subsystem (how multiple ROS nodes and or packages work together) should be documented in [projects](projects) (for entire ROS graphs, closed systems) and [subsystems](sybsystems) (a few ROS nodes working togehter which dont form a closed system) respectively.
Individual ROS packages should be documented in their individual README:s, found in the corresponding folder in the workspace.

The documentation of packages tries to mirror what is commonly seen on the ROS wiki.
For good and varied examples of package documentatios, see

- [twizy_webots](/workspace/src/twizy_webots)
- [twizy_msgs](/workspace/src/twizy_msgs)
- [piksi_multi_rtk](/workspace/src/piksi_multi_rtk)

# Writing Tests

It is always a good idea to write tests.
There are multiple kinds of tests which can run in ROS: Unit tests, node unit tests and node integration tests.
It is recommended to read about testing in ROS online.
For a good examples of packages that use tests see

- [twizy_control](/workspace/src/twizy_control)
- [twizy_localization](/workspace/src/twizy_localization)

# Licence

Licences for ROS packages are declared in `package.xml`.
We mostly use the [Apace 2.0 Licence](https://www.apache.org/licenses/LICENSE-2.0) which is the [recommended licence](https://docs.ros.org/en/foxy/Contributing/Migration-Guide.html#licensing) for newer versions of ROS.

# ROS Practices

Some general tips:

- Put launch files in [twizy_bringup](/workspace/src/twizy_bringup)
- Put message files in [twizy_msgs](/workspace/src/twizy_msgs)
- Put files and information related to the properties of the twizy (physical or simulated) in [twizy_description](/workspace/src/twizy_description)
- Make sure dependicies in `package.xml` file is kept up to date.
  You should even declare dependicies of other packages in the same workspace here.
- Copy the structure of already existing packages to get a good starting point when creating new packages
- Always use common ROS message types (e.g. from `std_msgs`, `geometry_msgs` or `sensor_msgs`) when you can!

## Recommended Reading

ROS delcares some of its norms in REP files, which are listed [https://ros.org/reps/rep-0000.html].
Some particular REPs to look through (only to get an idea of what they contain. Come back to them once they become relevant to you) are

- [Style Guide for Python Code](https://www.ros.org/reps/rep-0008.html)
- [Standard Units of Measure and Coordinate Conventions](https://www.ros.org/reps/rep-0103.html)
- [Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)

Other important pages to look through:

- [Installing Python scripts and modules](http://docs.ros.org/en/jade/api/catkin/html/howto/format2/installing_python.html)

# Writing Code (Finally...)

## Python

Precaution: I dont like python.
It is in grave need of a version 4 if you ask me.
That said...

Try your best to keep all code in this repo to python.

As ROS 1 only supports python and C++, these were the languages we initially had to choose between.
We chose to focus as much as possible on one language to keep the code base as uniform as possible.
That said, there can be times when python is just too slow.
If that is the case I first suggest you look for a package you can install to do the heavy work for you.
But we can forsee that there will come times when python can't be worked around, no matter how hard you try.

This beeing said (I'm not a python enthusiast remember), python has a lot of advantages:
- Short code which takes less time to write and to read
- A garbage collector
- Easy to install and use libraries and packages
- Computer programming education is moving away from C and java type languages and more towards scripting languages like python or node.
- Python is probably easier for beginners in programming to pick up than for example C++

### Code conventions

These are more even more like guidlines than rules compared to other thins mentioned on this page, but here goes:

- 80 character lines in python scripts
- autopep8 python formatter
- Four spaces as tabs

## Visual Studio Code

If you don't know what program to use for writing code, I recommend [Visual Studio Code](https://code.visualstudio.com/) which is available for all three major operating systems.
VS Code (or code) has many useful plugis.
These are some of the plugins and settings used in this project so far:

- "Markdown All in One" plugin for automatically generating ToC in documentation among other things

## Are We There Yet?

Yes we are.

1. Clone the project on your local computer (see [Installation](getting_started.md#installation)), or update existing local files with `git pull`.
2. Create your [branch](#branches) (`git checkout -b dev-my-amazing-new-feature`).
3. Make some changes.
4. Test those changes inside the newly created branch.
5. Push your new branch to the remote repo (github) with `git push -u origin dev-my-amazing-new-feature`
6. Open and merge a pull request from dev-my-amazing-new-featur to master through the [Pull requests](https://github.com/OssianEriksson/twizy-control-2021/pulls) tab

Phew...
This last step was all standard git stuff.
Google (or Duck Duck Go) is your friend.