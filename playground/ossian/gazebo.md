# Twizy simulation in gazebo

This is a test of gazebo made in the early stages of researching ROS and rviz before work on the Bachelor's thesis started for real.

# Running

To run a version of the simulation on your local computer you need to

1. [Install ROS](http://wiki.ros.org/ROS/Installation). 
   Get the full desktop install of ROS which includes rviz and gazebo, both neccessary to run this project. 
   The code has been tested on [ROS melodic](http://wiki.ros.org/melodic/Installation), which at the time of writing (early 2021) is the current operating system on the Twizy.

2. Clone this git repository and change into the cloned directory with
   ```sh
   git clone git@github.com:OssianEriksson/twizy-control-2021.git
   cd twizy-control-2021
   ```

3. Follow setup instructions from README:s of all packages contained in the [catkin workspace](catkin) (if there is a README included).
   At the time of writing the following sections are the relevant ones:
   * [twizy_description](catkin/src/twizy_description/README.md#getting-started)
   * [twizy_keyboard](catkin/src/twizy_keyboard/README.md#getting-started).
     Note that you may have to install pip with `sudo apt install python-pip` (or `sudo apt install python3-pip` if you are using newer versions of ROS/Ubuntu, in which case commands like `pip install ...` should be replaced by `pip3 install ...`) if you haven't already.

4. Re-enter the directory of the cloned git repository created in step 2 if you are not already inside it (`cd path/to/where/I/executed/git/clone/twizy-control-2021`).
   Start your ROS environement if you haven't already (the following commands are for a Linux install of ROS 1):
   ```sh
   source /opt/ros/<distro>/setup.bash
   roscore &
   ```
   Replace '\<distro\>' (including the '\<\>') with the name of your [ROS distribution](http://wiki.ros.org/Distributions) (e.g. melodic).
   Note that these commands need to be executed in the same shell (terminal) that you will run further commands from.

5. Change directory to the catkin workspace containing the projects source files, build the project and prepare for launching with
   ```sh
   cd playground/ossian/catkin
   catkin_make
   source devel/setup.bash
   ```

6. The entire stack, including gazebo simulaiton, rviz visualisaion and keyboard control can now be started with
   ```sh
   roslaunch twizy_keyboard simulation.launch
   ```
   If you would e.g. rather only start gazebo (no rviz or keyboard control) you can run
   ```sh
   roslaunch twizy_gazebo gazebo.launch
   ```
   There are some other launch files available as well, check out the `launch`-directory of all available packages.

(With reservation for typos and forgotten steps... 
Please ask if you have trouble: ossiane@student.chalmers.se.
If you discover that something is missing from this guide, feel free to add it yourself by branching off this branch and later re-merging your changes into this one.)