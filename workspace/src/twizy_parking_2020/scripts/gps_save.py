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
from std_msgs.msg import Float32MultiArray

x = []
y = []
angle = []
file_name = 'null'

def new_file():
    global file_name
    new_file_name = raw_input("Enter new file name including .txt: ")
    print(new_file_name)
    with open(new_file_name, 'w+') as f:
	init_angle = raw_input("what is the initial angle? ")
        f.write(str(init_angle) + "\n")
    file_name = new_file_name

def callback(data):
    if file_name == 'null':
	return
    global x,y,angle
    d = data.data
    x = float(d[0])
    y = float(d[1])
    angle = float(d[2])
    with open(file_name, 'a') as f:
        f.write(str(x) + "," + str(y) + "," + str(angle) + "\n")


def listener():
    rospy.init_node('namn', anonymous=True)
    rospy.Subscriber("GPS_pos", Float32MultiArray, callback)
    new_file()

    rospy.spin()

if __name__ == '__main__':
	listener()

