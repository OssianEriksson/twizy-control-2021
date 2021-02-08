#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib as mpl
import string
import math
import glob
import os
import rospy
from std_msgs.msg import Float64MultiArray

global GPS_history, d, distance

GPS_history = []
x = []
y = []
y_object = []
file_name = 'null'


def new_file():
    global file_name
    new_file_name = raw_input("Enter new file name for the map coordinates including .txt: ")
    print(new_file_name)
    with open(new_file_name, 'w+') as f:
        init_angle = raw_input("what is the p_length distance? ")
        f.write(str(init_angle) + "\n")
    file_name = new_file_name


def callback_gps(data_gps):
    global GPS_history
    # rospy.loginfo(rospy.get_caller_id() + 'i heard %s', data.data)
    GPS_history = data_gps.data  # stores: [local_x, local_y, local_angle]


def callback(data_ultrasonic):
    if file_name == 'null':
        return
    global x, y, y_object, x_object, GPS_history, dist_rearwheel
    print(data_ultrasonic.data)
    dist_behind, dist_rearwheel, dist_frontwheel = data_ultrasonic.data
    dist_rearwheel = float(dist_rearwheel)  # Rearwheel sensor in cm

    if dist_rearwheel > 450: dist_rearwheel = 450
    dist_rearwheel += 79  # 'y' length from gps to sensor
    dist_gps_sensor_x = 75  # 'x' length from gps to sensor
    x = float(GPS_history[0])
    y = float(GPS_history[1])
    angle = float(GPS_history[2])
    print(dist_rearwheel)
    extra_angle = -90
    y_object = float(y + dist_rearwheel / 100 * np.cos(angle) - dist_gps_sensor_x / 100 * np.sin(angle))  
    # Convert to meter from cm + distance / 100 * np.cos(angle)
    x_object = float(x - dist_gps_sensor_x / 100 * np.cos(angle) + dist_rearwheel / 100 * np.sin(angle))
    # Rotate object according to the car. Orthogonal to car's right side.

    with open(file_name, 'a') as f:
        f.write(str(x) + "," + str(y) + "," + str(x_object) + "," + str(y_object) + "\n")

    # For plotting gps coordinates on xy-axles

    size = 10
    plt.scatter(x_object, y_object, color="red", marker="x")
    plt.scatter(x, y, color="blue")
    plt.axis([2 * -size, size, -size, size])
    plt.ion()
    plt.show()
    plt.pause(0.01)  # Changed to 1/60 (60hertz) from 0.01. It is seconds


def listener():
    rospy.init_node('map_saver', anonymous=True)
    rospy.Subscriber("GPS_pos", Float64MultiArray, callback_gps, queue_size=1)
    rospy.Subscriber("ultrasonic", Float64MultiArray, callback, queue_size=1)
    new_file()

    rospy.spin()


if __name__ == '__main__':
    listener()

