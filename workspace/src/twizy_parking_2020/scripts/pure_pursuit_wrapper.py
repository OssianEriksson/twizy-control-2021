#!/usr/bin/env python
import pure_pursuit
import rospy
from std_msgs.msg import Float64MultiArray
from twizy.msg import car_control
from matplotlib import pyplot as plt
import numpy as np
import math

file_name = "garbage.txt"

def new_file():
    global file_name
    new_file_name = raw_input("Enter new file name including .txt: ")
    print(new_file_name)
    file_name = new_file_name

def write_pos(x, y):
    global file_name
    print(file_name, x ,y)
    with open(file_name, 'a') as f:
        f.write(str(x) + "," + str(y) + "\n")


class Map:
    def __init__(self,coordstart, coordbreak1, coordbreak2, coordend, twizydist, offset, GPS_y):
        self.coordstart = coordstart
        self.coordbreak1 = coordbreak1
        self.coordbreak2 = coordbreak2
        self.coordend = coordend
        self.twizydist = twizydist
        self.offset = offset
        self.GPS_y = GPS_y

    # all coordinates come from the GPS, twizydist from rear ultrasonic sensor
    def generateMap(self):
        offset_length = np.linspace(self.coordstart, self.coordbreak1 - 0.001, 20)
        extradots = np.linspace(self.coordbreak1, self.coordbreak1 + 0.1, 20)
        parkingspot_length = np.linspace(self.coordbreak1 + 0.1, self.coordbreak2, 20)
        end_length = np.linspace(self.coordbreak2, self.coordend, 20)

        parkingspotdepth = 2.5
        #  y = 25x + (distance . 25*offset)
        parkmap = {}

        for x in offset_length:
            parkmap[x] = self.twizydist + self.GPS_y
        for x in extradots:
            parkmap[x] = 25 * x + (self.twizydist - 25 * self.offset) + self.GPS_y
        for x in parkingspot_length:
            parkmap[x] = parkingspotdepth + self.twizydist + self.GPS_y
        for x in end_length:
            parkmap[x] = self.twizydist + self.GPS_y

        return parkmap


class Wrapper:

    def __init__(self):
        self.counter = 0
        self.path_is_ready = False
        self.GPS = None
        self.time = 0
        self.offset = None
        self.parking_length = None
        self.GPS_init_xpos = None
        self.init_counter = 0
        self.backsensor_dist = None
        self.collision_warning = False
	self.stop = False
	self.target_ind = None
        self.parking_map = {}


    def shutdown_hook(self):
        print('Goal reached!')


    def ros_plot(self, states):
        plt.cla()
        plt.plot(path.cx, path.cy, ".r", label="course")
        plt.plot(states.x, states.y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

	plt.subplots(1)
	plt.plot(states.x, states.e, ".r", label="Crosstrack Error")
	plt.xlabel("x[m]")
	plt.ylabel("e[m]")
	plt.grid(True)
	plt.show()
	

    def GPS_callback(self, msg):
        self.GPS = msg.data
        if self.path_is_ready and self.stop == False:
	    if self.init_counter < 1:
		self.GPS_init_xpos = msg.data[0]
		self.init_counter += 1
            #print(self.GPS[0], self.GPS[1], np.degrees(self.GPS[3]))
            target_speed = -1   # [m/s]
            # initial state
            state.update_from_gps(self.GPS, target_speed)  # yaw+3.14?
            lastIndex = len(path.cx) - 1

            states.append(self.time, state)
            write_pos(state.rear_x, state.rear_y)
            self.time += 1
            # Calc control input

            di, self.target_ind = pure_pursuit.pure_pursuit_steer_control(
                state, path, self.target_ind)


            di = -di*180/math.pi         # convert to degrees
            if di < -40:
                angle = -40		# capping the steering angle at +-40 degrees
            elif di > 40:
                angle = 40
            else:
                angle = di
            msg_to_publish.angle = angle
            msg_to_publish.speed = target_speed
            #print(angle,target_speed)
            pub.publish(msg_to_publish)



          	# stops if:
            dist_traveled_x = np.abs(msg.data[0] - self.GPS_init_xpos)
            if dist_traveled_x > self.offset + self.parking_length - 1  or self.collision_warning:
            #if self.target_ind > lastIndex:
	        print('Goal reached, shutting down')
                msg_to_publish.angle = 0
                msg_to_publish.speed = 0
                print(angle,target_speed)
                pub.publish(msg_to_publish)
                self.ros_plot(states)
		plt.subplots(2)
                plt.plot(list(self.parking_length.keys()), list(self.parking_map.values()), 'black')
		plt.grid(True)
		plt.show()
        #rospy.on_shutdown(self.shutdown_hook())

		self.stop = True
		exit()


    def path_callback(self, msg):
        path.path_generated =  msg.data[3]
        if path.is_path_generated() == 1 and self.counter < 1 and self.GPS != None:   #run once and only if gps signal is active
            a =  msg.data[0] # 0.8960
            b =  msg.data[1] # 0.6765
            c =  msg.data[2] # 0
            self.offset = msg.data[4]
            self.parking_length = msg.data[5]
            path.set_path(a, b, c, self.GPS[0] - 0.36 * math.cos(self.GPS[3]) , self.GPS[1] - 0.34 * math.sin(self.GPS[3]), self.GPS[3], self.offset, self.parking_length)	#generate path-------------------
            self.counter += 1
	    state.update_from_gps(self.GPS , -1)
	    self.target_ind, _ = path.search_target_index(state)

        #parameters for parking map
            start = 0
            p1 = self.offset + self.GPS[0]
            p2 = p1 + self.parking_length
            end = p2 + 3
            self.parking_map = Map(start, p1, p2, end, 1.5, self.offset, self.GPS[1]).generateMap()

            self.path_is_ready = True


    def ultrasonic_callback(self,msg):
	if self.path_is_ready:
            current_distances = msg.data #array from ultrasonic is saved in current distances
            dist_file.write(str(current_distances[0]) + ",") #write values from rear sensor to file


    def emergency_break(self):
	print ('collsion: ' , self.collision_warning)
        if self.path_is_ready: 
	   # initializes the custom message and makes sure the starting values are 0
	    dist = 0
	    dist_array = []
	    with open("distances.txt", "r") as dist_file:
			for line in dist_file.readlines():
			    f_list = [float(i) for i in line.split(",") if i.strip()]
			    dist_array += f_list
	    
	    if len(dist_array) > 15:
		    last_index = len(dist_array)
		    first_index = last_index - 10

		    for x in range(first_index, last_index):
			dist += dist_array[x]
		    print(dist/10)
		    if dist/10 < 30:
			self.collision_warning = True

	   
if __name__ == '__main__':
    new_file()
    rospy.init_node('pure_pursuit')
    pub = rospy.Publisher('controls', car_control, queue_size=5)

    msg_to_publish = car_control()

    rate = rospy.Rate(1)  # Adjust rate?
    path = pure_pursuit.TargetCourse()
    wrap = Wrapper()
    state = pure_pursuit.State()
    states = pure_pursuit.States()
    open('distances.txt', 'w').close()
    dist_file = open("distances.txt", "a")

    while not rospy.is_shutdown():
        rospy.Subscriber('GPS_pos', Float64MultiArray, wrap.GPS_callback)		# x_pos, y_pos, heading_angle
        rospy.Subscriber('path_planner', Float64MultiArray, wrap.path_callback)		# a, b, c, Boolean_done, offset, parking_length 
        #rospy.Subscriber('ultrasonic', Float64MultiArray, wrap.ultrasonic_callback)	# back_sensor, side_sensor_back, side_sensor_front
	#wrap.emergency_break()
        rate.sleep()
