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
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
x = []
y = []
file_name = 'gps_trash.txt'


def new_file():
    global file_name
    new_file_name = raw_input("Enter new file name including .txt: ")
    print(new_file_name)
    with open(new_file_name, 'w+') as f:
        f.write(" ---Remove this line---")
    file_name = new_file_name


def callback(data):
    global x, y
    d = data.data
    # gps_pos = d.split(",")
    gps_pos = data.data
    sens1 = float(gps_pos[0])
    sens2 = float(gps_pos[1])
    sens3 = float(gps_pos[2])
    sens1 = "%.10f" % sens1
    sens2 = "%.10f" % sens2
    sens3 = "%.10f" % sens3

    with open(file_name, 'a') as f:
        f.write(str(sens1) + "," + str(sens2) + "," + str(sens3) + "\n")


def listener():
    rospy.init_node('save_ultrasonic', anonymous=True)
    rospy.Subscriber("ultrasonic", Float64MultiArray, callback)
    new_file()
    #while (1 == 1):
     #   new_file()
    # plt.plot(x,y, label=GPS_route)
    # plt.show()

    rospy.spin()


# def test():
#    global x,y
#    with open('test_data_gps_circle.txt', 'r') as f:
#        f = f.readlines()
#        del f[-1]
#        for line in f:
#            lat = float(line.split(,)[0])
#            lon = float(line.split(,)[1])
#            x.append(lat)
#            y.append(lon)
#            lat = "%.10f" % lat
#            lon = "%.10f" % lon
#            print(lat + "," + lon)
#            with open('plot_gps_coords.txt', 'a') as f:
#                f.write(lat + "," + lon + "\n")
#
#        with open('plot_gps_coords.txt', 'a') as f:
#            f.write("\n" + "\n" + "\n" + "----------" "\n" + "\n" + "\n")
#
#    x_o = x[0]
#    y_o = y[0]
#    x = np.array(x)
#   y = np.array(y)
#    x = x - x_o
#    y = y - y_o
#    plt.plot(x,y)
#    plt.show()
#


if __name__ == '__main__':
    listener()
# test()

