#!/usr/bin/env python

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
import rospy
msg_to_publish = Float64MultiArray()





def talker():

    pub = rospy.Publisher('ultrasonic', Float64MultiArray , queue_size=2)
    a= 40
    b= 0
    c= 40
    while not rospy.is_shutdown():
        a -= 0.1
        msg_to_publish.data = [a, b, c]
        pub.publish(msg_to_publish)
	print([a, b, c])
	rate = rospy.Rate(10)
	rate.sleep()



if __name__ == '__main__':
    rospy.init_node('ultrasonicc')
    talker()
