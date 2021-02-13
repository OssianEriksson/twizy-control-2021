# Tests

About testing code.

# Table of Contents

- [Running Tests](#running-tests)
  * [Run all catkin tests](#run-all-catkin-tests)
  * [Run specific ROS test](#run-specific-ros-test)


# Running Tests

## Run all catkin tests

To run tests of all ROS-packages which have registred tests in catkin, use
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