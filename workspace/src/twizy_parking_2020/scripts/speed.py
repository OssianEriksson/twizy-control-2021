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
import time

x = []
y = []
speed = []
file_name = 'null'
prev_x = 0
prev_y = 0
prev_time = 0


def new_file():
    global file_name
    new_file_name = raw_input("Enter new file name including .txt: ")
    print(new_file_name)
    file_name = new_file_name

def dist(x1, y1, x2, y2):
    return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def calc_speed(x, y):
    global prev_x, prev_y, prev_time
    speed  = dist(prev_x, prev_y, x, y)/(time.time() - prev_time)
    if speed == 0:
        return
    prev_time = time.time()
    prev_x = x
    prev_y = y

    return speed


def callback(data):
    if file_name == 'null':
	return
    global x, y, speed
    d = data.data
    x = float(d[0])
    y = float(d[1])
    speed = calc_speed(x, y)
    if speed == None:
        return
    print(speed)
    with open(file_name, 'a') as f:
        f.write(str(speed) + "\n")


def listener():
    rospy.init_node('Speed', anonymous=True)
    rospy.Subscriber("GPS_pos", Float64MultiArray, callback)
    new_file()
    rospy.Rate(1)
    rospy.spin()

if __name__ == '__main__':
	listener()
