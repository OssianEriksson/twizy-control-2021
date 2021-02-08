#!/usr/bin/env python
# Software License Agreement (BSD License)

## Simple talker demo that listens to std_msgs/Strings published
## to the 'ultrasonic' topic
import math
import numpy as np
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
# from std_msgs.msg import Float32MultiArray  # Unsure if the gps_calc uses 32 or 64. Edit: NOT used
from twizy.msg import car_control
import time

global GPS_history  # Probably horrible use of global variables, but I can't be asked anymore
global all_distances
global all_distances_with_gps
global pspot_distances
global object_distances
global startpoint
global endpoint
global parking_length
global offset
global distance_to_car
mapping_variables = Float64MultiArray()
distances = Float64MultiArray()
global mapping_state
global safety_distance

GPS_history = []
all_distances = []
all_distances_with_gps = []
pspot_distances = []
object_distances = []
startpoint = [0, ()]  # Changed both to None/[] (after changing from list to array)
endpoint = [0, ()]
x_distance = 0
mapping_state = True


def callback_ultrasonicsensor(data):
    # rospy.loginfo(rospy.get_caller_id() + 'i heard %s', data.data)
    current_distances = data  # [behind, rearwheel, frontwheel] distances
    mapping(current_distances)
    # Perhaps a need to append values before going into mapping


def callback_gps(data):
    # rospy.loginfo(rospy.get_caller_id() + 'i heard %s', data.data)
    #GPS_history.append(data.data)  # stores: [local_x, local_y, local_angle]
    # NEW: tuple unpacking
    local_x, local_y, local_angle, zero_one = data.data
    temp_GPS_history = [local_x, local_y, local_angle]
    GPS_history.append(temp_GPS_history)  # stores: [local_x, local_y, local_angle]


def mapping(current_distances):
    """Function to map the surrounding to the right of the car"""
    global parking_length
    parking_length = 0
    global offset
    offset = 0
    global distance_to_car
    distance_to_car = 0
    global endpoint
    global startpoint
    global mapping_state
    global x_distance
    global safety_distance

    # Adjustments to measure distance from GPS-unit to the object (not sensor to object)
    # distance_gps_rearwheelsensor = XX
    # object_coordinates = GPS_history[1] + distance/100
    # y-distance is orthogonal if driving straight forward. If not, *rotation matrix(current_angle)

    # Distances are measured within the cone of 15 degrees --> can detect the same object at different spots
    # Example: detects the endpoint earlier than when the sensor is orthogonal to it, for a longer time for startpoint

    # A need to adjust the coordinate for start/endpoint is brought up. Trigonometry for the distance.
    cone_angle = 15*(np.pi/180) #/ 2   Radians (Divided by 2 due to sensor precision in wider area)
    # adjusted_distance = np.cos(cone_angle)* the_measured_distance
    # OR: adjust requirement for parking_length
    # DECISION: Adjusting coordinates for startpoint and endpoint, does not affect other calculations


    position_change = 100  # Init to enter mapping (meter)
    all_distances.append(current_distances.data)
    all_distances_with_gps.append([current_distances.data, GPS_history[-1]])
    if len(all_distances) < 3:
        x_distance = current_distances.data[1]
        safety_distance = x_distance + 250  # For Assessing if the distance is enough to be a empty parkingspace
        #safety_distance -= 100   Temporarily reduced
        print(x_distance)
        print("Safety distance:")
        print(safety_distance)
    elif len(all_distances) > 5 and mapping_state:  # == True. Only check mapping after 5 values have been recorded
        controller()  # Controls the speed during the mapping sequence. 0 if parkingspot found (endpoint)

        print(all_distances[-1][1])
        if all_distances[-1][1] > safety_distance and all_distances[-2][1] > safety_distance \
                and all_distances[-3][1] > safety_distance:
            # Pair the sensordistances with a gps-position (imaginary) if large enough (empty parkingspot)
            if all_distances[-4][1] <= safety_distance and all_distances[-5][1] <= safety_distance \
                    and all_distances[-6][1] <= safety_distance and endpoint == [0, ()]:
                # If it is the first time, include the previous ok measurements (3 in a row)
                pspot_distances.append([all_distances[-3][1], GPS_history[-3]])
                pspot_distances.append([all_distances[-2][1], GPS_history[-2]])
                print("\nSTARTPOINT")
                startpoint = [all_distances[-3][1], GPS_history[-3]]  # Instead of a list, it is now a value
                print(startpoint)
                adjusted_startdistance = startpoint[1][0] + np.sin(cone_angle) * all_distances[-4][1]/100
                # sin(angle) * (latest distance smaller than the safety distance)
                #startpoint = [startpoint[0], [adjusted_startdistance, startpoint[1][1], startpoint[1][2] ] ]
                # CHANGE!!!
                startpoint = pspot_distances[0]
                startpoint[1][0] = adjusted_startdistance

                print(startpoint)
                # Add the extra distance. Forward for the car is -x, therefore the +
                # Car is moving forward in the local coordinate system. Forward from GPS-reset = -x
                # If not moving only in x-direction, it needs adjustments

            pspot_distances.append([all_distances[-1][1], GPS_history[-1]])

        elif all_distances[-1][1] <= safety_distance and all_distances[-2][1] <= safety_distance \
                and all_distances[-3][1] <= safety_distance:

            print("first test")
            if all_distances[-4][1] > safety_distance and all_distances[-5][1] > safety_distance \
                and all_distances[-6][1] > safety_distance:

                print("Checking if startpoint and endpoint has been found")
                print(startpoint)
                print(endpoint)
                endpoint = pspot_distances[-1]  # Instead of a list, it is now a value
		        # Changed above
                if endpoint != [0, ()] and startpoint != [0, ()]:  # TEST: Changed [] to [0,()]
                    # Check if there is an endpoint
                    endpoint = [all_distances[-4][1], GPS_history[-4]]  # Instead of a list, it is now a value
                    # Should only be one anyways
                    print("\nENDPOINT")
                    print(endpoint)
                    adjusted_enddistance = endpoint[1][0] - np.sin(cone_angle) * all_distances[-1][1] / 100
                    # sin(angle) * (latest distance smaller than the safety distance). /100 due to cm --> m
                    endpoint = [endpoint[0], [adjusted_enddistance, endpoint[1][1], endpoint[1][2]]]

                    # Subtract the extra distance. Forward for the car is -x, therefore the -
                    # Same as with startpoint, but add instead
                    print(endpoint)

                    print("\nCheck if length is 5 or greater")
                    print(math.hypot(endpoint[1][0] - startpoint[1][0], endpoint[1][1] - startpoint[1][1]))

                    if math.hypot(endpoint[1][0]-startpoint[1][0], endpoint[1][1]-startpoint[1][1]) < 5:
                        # Check if the length is enough. If not, reset it
                        # !!!
                        # CHANGED to 2 meter
                        # !!!
                        print(endpoint)
                        endpoint = [0, ()]
                        startpoint = [0, ()]  # Reset startpoint due to the failure
                        del pspot_distances[:]  # Clear approved distances if fail
                        print("\nENDPOINT Failure")
                        print(endpoint)
            elif all_distances[-4][1] < safety_distance and all_distances[-5][1] < safety_distance \
                and all_distances[-6][1] < safety_distance and all_distances[-7][1] < safety_distance and all_distances[-8][1] < safety_distance and all_distances[-9][1] < safety_distance:
		        # Added three and statements
                startpoint = [0, ()]  # Reset startpoint if there is an approved object
                print("APPROVED OBJECT, resetting startpoint!")
                object_distances.append([all_distances[-4][1], GPS_history[-4]])  # Approved object, append to list
                object_distances.append([all_distances[-5][1], GPS_history[-6]])
                object_distances.append([all_distances[-6][1], GPS_history[-6]])


        # pythagoras of deltax and deltay first[1] for gps, second [0] or [1] for x or y coordinate
        # Only append one endpoint, overwrites the old endpoint

        # !! GPS coordinates unit is meter.
        if endpoint != [0, ()] and startpoint != [0, ()]:
            print("BOTH endpoint and startpoint found")
            # Continue to update the offset & distance to car if both endpoint and startpoint are found
            parking_length = math.hypot(endpoint[1][0] - startpoint[1][0], endpoint[1][1] - startpoint[1][1])
            offset = math.hypot(GPS_history[-1][0] - endpoint[1][0], GPS_history[-1][1] - endpoint[1][1])
            distance_to_car = all_distances[-1][1]/100  # Latest distance measured from rearwheel
            print(parking_length)
            print(offset)
            print(distance_to_car)
            position_change = math.hypot(GPS_history[-10][0] - GPS_history[-1][0], GPS_history[-10][1] - GPS_history[-1][1])
            while(1):  # Delay to get a better/larger offset with speed = 1km/h
                time.sleep(1)  # 1/3.6 meter
                break

            controller()
            if position_change < 0.05:
                talker_mapping_variables(1)  # Index tells talker if the car is standing still or not. 1 = True
                mapping_state = False
                print("IN position change; standing still")
                with open('mapping550magain1.txt', 'w+') as f:  # Write the important values to a textfile
                    print("Creating mapping_distances.txt")
                    f.write(str(pspot_distances) + "\n")
                    f.write(str(object_distances) + "\n")
                    f.write(str(startpoint) + "\n")
                    f.write(str(endpoint) + "\n")
                while(1):
			        talker_mapping_variables(1)  # Keep looping since mapping is done
            else:
                talker_mapping_variables(0)  # Still moving
                print("Still moving")

    # TODO:
    # Decide when to stop mapping script. Now True/false mapping_state
    # Fix distances for the objects coordinates
    # Maybe change != [0, ()] to != if len(startpoint[1][0]) > 0
    # Doublecheck the filter, only the behind sensor seems so filter 450 cm

    # Of the mapping variables, only the offset is to be updated after an endpoint is found
    # if endpoint != [0, ()]:
    #    talker()  # Added this call to the talker to publish mapping variables []


def controller():
    global endpoint
    pub_steering = rospy.Publisher('controls', car_control, queue_size=1)
    rate = rospy.Rate(60)
    msg_to_publish = car_control()

    # Drive forward 1 km/h until endpoint is found (i.e. done mapping, spot found)
    msg_to_publish.angle = 0
    #msg_to_publish.speed = 1
    if endpoint != [0, ()]:
        msg_to_publish.speed = 0
        print("\nSPEED 0")
    else: # endpoint == [0, ()]:
        msg_to_publish.speed = 1

    pub_steering.publish(msg_to_publish)
    rate.sleep()


# Added the following function monday afternoon
def talker_mapping_variables(standstill):
    global parking_length
    global offset
    global distance_to_car
    #print("\nIN TALKER")

    pub_mapping_var = rospy.Publisher('mapping_var', Float64MultiArray, queue_size=1)
    # Error with type Float when publishing a list..?
    rate_talker = rospy.Rate(60)
    mapping_variables.data = [parking_length, offset, distance_to_car, standstill]
    #print(mapping_variables)
    pub_mapping_var.publish(mapping_variables)
    rate_talker.sleep()

def listener():
    rospy.Subscriber('GPS_pos', Float64MultiArray, callback_gps)  # Changed here to actual gps topic
    rospy.Subscriber('ultrasonic', Float64MultiArray, callback_ultrasonicsensor)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('mapping', anonymous=True)
    listener()


