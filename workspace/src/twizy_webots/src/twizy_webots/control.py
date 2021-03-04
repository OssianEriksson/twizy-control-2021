import rospy
import rospkg

from math import tan, atan
import yaml
import os

from webots_ros.srv import set_float
from std_msgs.msg import String
from twizy_msgs.msg import CarControl

import twizy_webots.util as util


def get_position_control_service(model_name, device):
    """
    Returns a set_position ServiceProxy for interfacing with the webots device
    with name device of robot with name model_name
    """

    return rospy.ServiceProxy('/{}/{}/set_position'.format(model_name, device),
                              set_float)


def get_velocity_control_service(model_name, device):
    """
    Disables position control and returns a set_velocity ServiceProxy for
    interfacing with the webots device with name device of robot with name
    model_name
    """

    # Disable position control
    set_position = get_position_control_service(model_name, device)
    set_position.wait_for_service()
    set_position(float('inf'))  # This is what actually does the disabling
    set_position.close()  # Close isn't really needed here,
                          # included for good measure

    srv = rospy.ServiceProxy('/{}/{}/set_velocity'.format(model_name, device),
                                 set_float)

    srv.wait_for_service()

    # Setting the speed to 0 initially is needed since the Twizy otherwise just
    # drives away until it recieves it first CarControl message
    srv(0.0)

    return srv


def main():
    # Initialize ROS node
    rospy.init_node('controller')

    # Get the models's name from topic advertised by webots
    model_name = util.model_name()

    # Read parameters from ROS parameter server
    wheel_radius = rospy.get_param('/twizy_properties/wheel/radius')
    max_speed = rospy.get_param('/twizy_properties/wheel/max_speed')
    max_angle = rospy.get_param('/twizy_properties/wheel/max_steering_angle')
    wheelbase = rospy.get_param('/twizy_properties/chassis/wheelbase')
    track = rospy.get_param('/twizy_properties/chassis/track')

    # Initalize services
    left_steering = get_position_control_service(model_name,
                                                 'front_left_hinge_motor')
    right_steering = get_position_control_service(model_name,
                                                  'front_right_hinge_motor')
    rear_left_weel = get_velocity_control_service(model_name,
                                                  'rear_left_wheel_motor')
    rear_right_weel = get_velocity_control_service(model_name,
                                                   'rear_right_wheel_motor')

    def cb_control(msg):
        """
        Sets simulation model's steering angle and velocity
        """

        # Clamp value within allowed bounds
        angle = max(min(msg.angle, max_angle), -max_angle)
        speed = max(min(msg.speed, max_speed), -max_speed)

        # Convert from linear to angular velocity
        angular_vel = speed / wheel_radius

        # Ackermann steering
        centerline_to_intersection = (
            wheelbase / tan(angle)) if abs(angle) > 1e-10 else float('inf')
        angle_l = atan(wheelbase / (centerline_to_intersection - track / 2))
        angle_r = atan(wheelbase / (centerline_to_intersection + track / 2))

        # Set steering angles
        if not left_steering(angle_l):
            rospy.logwarn('Unable to set left side steering angle')
        if not right_steering(angle_r):
            rospy.logwarn('Unable to set right side steering angle')

        # Set rear wheel speeds
        if not rear_left_weel(angular_vel):
            rospy.logwarn('Unable to set rear left wheel speed')
        if not rear_right_weel(angular_vel):
            rospy.logwarn('Unable to set rear left wheel speed')

    # Subscribe to topic where reference control values will be published
    # Increasing the queue_size will create INSANE amount of accumulative input
    # lag for some reason (took way too long to figure that out...)
    rospy.Subscriber('car_control', CarControl, cb_control, queue_size=1)

    # Wait for shutdown
    rospy.spin()
