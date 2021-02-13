#!/usr/bin/python

import rospy
import xml
import math

from std_msgs.msg import Float64


class AckermannNode:

    pub_left_wheel = rospy.Publisher(
        '/twizy/left_wheel_angle_controller/command', Float64, queue_size=1)
    pub_right_wheel = rospy.Publisher(
        '/twizy/right_wheel_angle_controller/command', Float64, queue_size=1)

    def __init__(self, vehicle_params):
        self.wheelbase = vehicle_params['wheelbase']
        self.track = vehicle_params['track']
        self.max_angle = math.atan(2 * self.wheelbase / self.track) - 1e-10

        rospy.Subscriber('/twizy/ackermann_controller/command',
                         Float64, self.convert_to_ackermann)

    def convert_to_ackermann(self, input):
        angle = min(max(input.data, -self.max_angle), self.max_angle)

        centerline_to_intersection = self.wheelbase / \
            math.tan(angle) if abs(angle) > 1e-10 else float('inf')

        left_angle = math.atan(
            self.wheelbase / (centerline_to_intersection - self.track / 2))
        right_angle = math.atan(
            self.wheelbase / (centerline_to_intersection + self.track / 2))

        self.pub_left_wheel.publish(left_angle)
        self.pub_right_wheel.publish(left_angle)


def determine_vehicle_params():
    def log_params_warning(msg, params):
        rospy.logwarn(
            '{}\nUnable to determine model parameters. Using default values: {}'.format(msg, params))

        return params

    params = {'wheelbase': 1.5, 'track': 1}

    if not rospy.has_param('robot_description'):
        return log_params_warning('No robot_description parameter set', params)

    try:
        urdf = xml.etree.ElementTree.fromstring(
            rospy.get_param('robot_description'))
    except xml.etree.ElementTree.ParseError as e:
        return log_params_warning('Unable to parse robot_description xml: {}'.format(str(e)), params)

    for ackermann_tag in urdf.findall('twizy_ackermann'):
        for param_tag in ackermann_tag:
            if param_tag.tag in params:
                try:
                    params[param_tag.tag] = float(param_tag.text)
                except ValueError as e:
                    rospy.logwarn(
                        'Expected {} to contain value of type float, ignoring.'.format(param_tag.tag))
            else:
                rospy.logwarn(
                    'Unrecognized element {}, ignoring'.format(param_tag.tag))

    return params


if __name__ == '__main__':
    rospy.init_node('twizy_ackermann_converter')
    node = AckermannNode(determine_vehicle_params())
    rospy.spin()
