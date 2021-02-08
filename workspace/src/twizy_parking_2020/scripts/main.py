#!/usr/bin/env python
import pure_pursuit
import rospy
from std_msgs.msg import Float32MultiArray
from twizy.msg import car_control
from matplotlib import pyplot as plt
import numpy as np
import math


class Main:

    def __init__(self):
        self.counter = 0
        self.path_is_ready = False
        self.GPS = None
        self.time = 0

    def shutdown_hook(self):
        print('Goal reached!')


    def ros_plot(self, state, states):
        plt.cla()
        plt.plot(path.cx, path.cy, ".r", label="course")
        plt.plot(states.x, states.y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        plt.subplots(1)
        plt.plot(states.t, [iv * 3.6 for iv in states.v], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        plt.show()

    def GPS_callback(self, msg):
        self.GPS = msg.data
        print(self.path_is_ready)
        if self.path_is_ready:


            target_speed = -1 / 3.6  # [m/s]

            # initial state
            state.update_from_gps(self.GPS, target_speed)  # yaw+3.14?
            lastIndex = len(path.cx) - 1
            target_ind, _ = path.search_target_index(state)


            states.append(self.time, state)
            self.time += 1
            # Calc control input
            ai = pure_pursuit.proportional_control(target_speed, state.v)
            di, target_ind = pure_pursuit.pure_pursuit_steer_control(
                state, path, target_ind)
            state.update_from_gps(self.GPS, ai)
            di = di*180/math.pi         # convert to degrees
            if di < -40:
                angle = -40
            elif di > 40:
                angle = 40
            else:
                angle = di
            msg_to_publish.angle = angle
            msg_to_publish.speed = target_speed
            print(angle,target_speed)
            pub.publish(msg_to_publish)

            #if lastIndex <= target_ind:

            if self.GPS[0] > 7:             # Cancels at gps.x > 7 atm
                print('mmmmmmmmmmmmmmmmmmmmmmmmmmmmmm')
                self.ros_plot(state, states)
                rospy.on_shutdown(self.shutdown_hook())

    def path_callback(self, msg):  # fix callback when mapping is done
        #print(msg.data[3])
        path.path_generated =  msg.data[3]
        if path.is_path_generated() == 1 and self.counter < 1 and self.GPS != None:  # TODO: only execute once!
            a =  msg.data[0] # 0.8960
            b =  msg.data[1] # 0.6765
            c =  msg.data[2] # 0
            path.set_path(a, b, c, self.GPS[0], self.GPS[1], self.GPS[2])
            self.counter += 1
            self.path_is_ready = True

if __name__ == '__main__':

    rospy.init_node('pure_pursuit')
    pub = rospy.Publisher('controls', car_control, queue_size=5)
    msg_to_publish = car_control
    rate = rospy.Rate(10)  # Adjust rate?
    path = pure_pursuit.TargetCourse()
    main = Main()

    while not rospy.is_shutdown():
        rospy.Subscriber('GPS_pos', Float32MultiArray, wrap.GPS_callback)
        rospy.Subscriber('path_planner', Float32MultiArray, wrap.path_callback)
        rate.sleep()
