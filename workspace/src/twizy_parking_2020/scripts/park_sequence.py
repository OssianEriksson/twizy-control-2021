#!/usr/bin/env python

# This node recives values from "mapping", calculates the angle and publishes this to the topic "controls". It is used to manually drive the car.
import rospy
from twizy.msg import car_control
import math
import time
from std_msgs.msg import Float64MultiArray
global radius1
global angle
global t0

# initierar nagra variabler med exempelvarden
t0 = time.time()
angle = 0
bredd = 1.08
langdmellanaxlar = 1.72
theta1deg = 40
theta1 = theta1deg*((2*math.pi)/360) # i radianer
theta2cot = (math.cos(theta1))/(math.sin(theta1))-bredd/langdmellanaxlar
theta2 = theta1 # (math.pi/2)-math.atan(theta2cot)

# formler for olka radier och for att rakna ut var cirkelbage
radie1 = langdmellanaxlar/math.tan(theta1) + bredd/2
radie2 = langdmellanaxlar/math.tan(theta2) + bredd/2
cb = radie1
ac = radie1 + radie2
acb = ac**2-cb**2
xd = math.sqrt(acb)
alfa = math.atan(xd/radie1) # galet har ocksa tror jag
beta = alfa
langdcirkel1 = alfa*(radie1-(bredd/2))
langdcirkel2 = beta*(radie2+(bredd/2)) # obs theta ar fel

# main loop, creates node and publishes the speed and angle to "controls"
def talker2():
	pub = rospy.Publisher('controls', car_control, queue_size=1) # Car_control??
	# pub = rospy.Publisher('controls', Float64MultiArray, queue_size=5)
	rospy.init_node('talker2', anonymous=True)
	rate = rospy.Rate(60) # 10hz
	# initializes the custom message and makes sure the starting values are 0
	msg = car_control() #  Tror att vi kor pa Float62 istallet
	# msg = Float64MultiArray
	print("Lets go to the end of the world and beyond")

	while not rospy.is_shutdown():
		angle = 0
		speed = -1 # km/h?
		# initiera timer
		t1 = time.time()
		tdiff = t1 - t0
		time_turn1 = langdcirkel1/(speed/3.6)*-1
		time_turn1 += 1.5  # Added a delay of the "stigtid"
		time_turn2 = langdcirkel2/(speed/3.6)*-1 + 1.5
		# if-satser for olika vinklar och olika tider
		if tdiff < time_turn1:
			angle = 40
		if tdiff > time_turn1 and tdiff < time_turn1+time_turn2:
			angle = -40
		if tdiff > time_turn1+time_turn2:
			speed = 0
			angle = 0
	        print("speed:", speed, "angle", angle, "variabelCheck:", time_turn2)

	        msg.angle = angle
	        msg.speed = speed
	        pub.publish(msg)
	        rate.sleep()

if __name__ == '__main__':
    try:
        talker2()
    except rospy.ROSInterruptException:
        pass

