#!/usr/bin/env python
# Software License Agreement (BSD License)

# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'ultrasonic' topic

#import math
import rospy
import random
from std_msgs.msg import Float64MultiArray

global counter
global b
global c
global d

a = Float64MultiArray()


def talker():
    pub = rospy.Publisher('ultrasonic', Float64MultiArray, queue_size=10)
    rospy.init_node('testtalker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    counter=0

    while not rospy.is_shutdown():
        rospy.loginfo(a)
        pub.publish(a)
        rate.sleep()
        counter+=1
        c = random.randrange(5)
        d = random.randrange(5)
        if counter<5:
            b = 1
        elif counter==5:
            b= 5
        elif counter>5 and 30>counter:
            b=1
        elif counter>=30 and 50>counter:
            b=3.5
        elif 50<counter:
            b=1

        a.data=[b, c, d]

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

