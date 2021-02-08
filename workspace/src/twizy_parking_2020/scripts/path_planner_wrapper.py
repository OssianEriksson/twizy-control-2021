#!/usr/bin/env python
import path_planner
import rospy
from std_msgs.msg import Float64MultiArray


class Wrapper:

    def shutdown_hook(self):
        print('Goal reached!')

    def mapper_callback(self, msg):

        if msg.data[3]:
            offset = msg.data[1]
            parking_length = msg.data[0]
            distance = msg.data[2] + 0.41
            current = path_planner.Coordinate(0, 0)
            goal = path_planner.Coordinate(offset + parking_length - 1, distance + 1)

            start = 0
            p1 = offset
            p2 = offset + parking_length
            end = offset + parking_length + 3

            parking_map = path_planner.Map(start, p1, p2, end, distance, offset).generateMap()

            a, b, c = path_planner.path(current, goal, parking_map)
            msg_to_publish.data = [a, b, c, 1, offset, parking_length]
	    while(1):
	        pub.publish(msg_to_publish)
	        print(a, b, c, msg.data[0], msg.data[1], msg.data[2], msg.data[3])
        else:
            print('Mapping not yet done!')
            msg_to_publish.data = [0, 0, 0, 0, 0, 0]
 	    pub.publish(msg_to_publish)


if __name__ == '__main__':

    rospy.init_node('PathPlanner')
    pub = rospy.Publisher('path_planner', Float64MultiArray, queue_size=5)
    msg_to_publish = Float64MultiArray()
    
    rate = rospy.Rate(1)  # Adjust rate?
    wrap = Wrapper()
    while not rospy.is_shutdown():
        rospy.Subscriber('mapping_var', Float64MultiArray, wrap.mapper_callback)
        #msg_to_publish.data = [1.1679, 0.9473,0.0,1, 1, 6]
        #print(msg_to_publish.data[0], msg_to_publish.data[1], msg_to_publish.data[2], msg_to_publish.data[3], msg_to_publish.data[4], msg_to_publish.data[5])
        #pub.publish(msg_to_publish)
        rate.sleep()
