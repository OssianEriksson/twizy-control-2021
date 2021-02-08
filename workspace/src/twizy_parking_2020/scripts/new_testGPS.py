#!/usr/bin/env python

import rospy
# import random
from std_msgs.msg import Float64MultiArray
global counter
a = Float64MultiArray()
global mapping_variables



def callback_mapping(data):
    print("2")

    mapping_variables = data.data

    rate = rospy.Rate(60) # 10hz

    indicator = 0
    if mapping_variables != []:
	indicator = 0
    else:
	indicator = 1  # Endpoint found, do not update gps position
    talker(indicator)

    rate.sleep()


def talker(indicator):
    print("3")


    counter=0
    pub = rospy.Publisher('GPS_pos', Float64MultiArray, queue_size=1)
    if indicator == 0:
        counter+=1
        b=counter
        c=2
        d=2
	e=0
	a.data=[b, c, d, e]
    else:
        #counter+=1
        b=counter
        c=2
        d=2
	e=0
	a.data=[b, c, d, e]

    rospy.loginfo(a)
    pub.publish(a)
    rate = rospy.Rate(10) # 10hz
    rate.sleep()


def listener():
    global indicator
    indicator = 0
    talker(indicator)
    if a.data[1] > 30:
	rospy.Subscriber('mapping_var', Float64MultiArray, callback_mapping)
    talker(indicator)
    print("1")
    rospy.spin()

if __name__ == '__main__':
    global indicator
    indicator = 0
    rospy.init_node('testGPS', anonymous=True)
    listener()
