# Usage Instructions <!-- omit in toc -->

# Table of Contents <!-- omit in toc -->

- [Building](#building)
- [Tests](#tests)
  - [Run all catkin tests](#run-all-catkin-tests)
  - [Run specific ROS test](#run-specific-ros-test)

# Building

Run
```sh
catkin_make
source devel/setup.bash
```
on linux or equivalent on windows.

# Tests

## Run all catkin tests

To run tests of all ROS-packages which have registred tests in catkin first make sure you have [built the project](#Building) once before and sourced `setup.bash`, then use
```sh
catkin_make run_tests
```

To review the latest tests results, you can use
```sh
catkin_test_results [--verbose]
```

## Run specific ROS test

To run a specific ROS test, use
```sh
rostest <package_name> <test_name> [--text]
```