# Usage Instructions <!-- omit in toc -->

# Table of Contents <!-- omit in toc -->

- [Building](#building)
- [Tests](#tests)
  - [Run All Catkin Tests](#run-all-catkin-tests)
  - [Run Specific ROS test](#run-specific-ros-test)
  - [Recommended Reading](#recommended-reading)
- [Running](#running)
  - [Running Individual ROS nodes](#running-individual-ros-nodes)
  - [Running Project Stacks](#running-project-stacks)

# Building

Before you can run any code you need to compile or build it.
Currently the code in this repo is based on ROS 1, so the workspace is handeled by catkin.

To build the code in the workspace, first change directory to the catkin workspace:
```sh
cd workspace
```
Then run
```sh
catkin_make
source devel/setup.bash
```
on linux or equivalent on windows.

# Tests

Tests help avoid errors in code.
There are both unit tests and integration tests available in this repo.
Before merging branches to master you should make sure all tests are able to complete.

## Run All Catkin Tests

To run tests of all ROS-packages which have registred tests in catkin first make sure you have [built the project](#building) once before and sourced `setup.bash`, then use
```sh
catkin_make run_tests
```

To review the latest tests results, you can use
```sh
catkin_test_results [--verbose]
```

## Run Specific ROS test

To run a specific ROS test, use
```sh
rostest <package_name> <test_name> [--text]
```

## Recommended Reading

- [Running unit tests with catkin](http://docs.ros.org/en/indigo/api/catkin/html/howto/format2/run_tests.html)
- [rostest](http://wiki.ros.org/rostest)
- [Automatic Testing with ROS](http://wiki.ros.org/Quality/Tutorials/UnitTesting)

# Running

## Running Individual ROS nodes

To view running instructions for inidividual ROS nodes please see the package specific README:s located in [/workspace/src](/workspace/src).

## Running Project Stacks

Please refer to [Starting and System Details](README.md#starting-and-system-details).

All ROS launch files of importance to the end user are placed in the main package specific to that project, for example [twizy_control_2021](/workspace/src/twizy_control_2021) for files relating to [Autonomous Renault Twizy - Autonomous Path Planning](projects/2021/autonomous_path_planning.md).