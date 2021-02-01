# twizy_keyboard

ROS node to control the Twizy with a keyboard

# Getting started

This node requires the python package `pynput` which can be installed with
```sh
pip install pynput
```
If you are using linux, you may have to give yourself access to the `/dev/input` device files.
On Debian and Ubuntu this can be done by adding yourself to the `input` group:
```sh
sudo usermod -a -G input $USER
```
Note that you have to log out and back in again for your addition to the group to take effect.