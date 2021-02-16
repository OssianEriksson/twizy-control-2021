#!/usr/bin/env python

import argparse
import random
import socket

import unittest
import rostest
import rospy

from twizy_msgs.msg import CarControl
from twizy_parking_2020.msg import car_control

from canlib import canlib, Frame


PKG = 'twizy_control'
NAME = 'test_can'
DESCRIPTION = 'Test control node'


class TestCAN(unittest.TestCase):

    def _tcp_connection_possible(self, host, port):
        """
        Opens and then immediately closes a TCP socket, just to see if it is
        possible to connect to the given host and port.
        """

        # Try to connect to (host, port)
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(1)
            s.connect((host, port))

            # Close the socket immediately
            s.close()

            return True
        except:
            return False

    def _running_on_twizy(self):
        """
        Check if script is run on the Twizy's computer
        """

        # We deem that we are running on the twizy if we can contact either the
        # left or right GNSS reciever
        return (self._tcp_connection_possible('192.168.0.222', 55555) or
                self._tcp_connection_possible('192.168.0.223', 55555))

    def setUp(self):
        # If we are running on the Twizy - don't! Abort the test!
        if self._running_on_twizy():
            self.fail('This test should not run on the real car as the test '
                      'involves writing to the CAN-bus which if we are running '
                      'on the real car and not on a virtual device might cause '
                      'the car to start moving about in a generally unsafe '
                      'manner.')

        # Initialize ROS node
        rospy.init_node('test_tcp')

        # Set up two publishers, one for messages to the old node (from 2020),
        # one for messages to the current node
        self.pub_old = rospy.Publisher('/old/controls', car_control,
                                       queue_size=queue_size, latch=True)
        self.pub_new = rospy.Publisher('/new/twizy_control', CarControl,
                                       queue_size=queue_size, latch=True)

        # Wait for subscribers to appear on each of the published topics
        t = rospy.get_time()
        while (self.pub_old.get_num_connections() < 1 and
               self.pub_new.get_num_connections() < 1):
            if rospy.get_time() > t + connect_timeout:
                self.fail(('Subscribers did not connect to topics in time. '
                           'Set timeout with the --connect-timeout option. '
                           'The current value is {}. \n\nIMPORTANT: To run '
                           'tests you need to start a Kvaser virtual device '
                           'with at least two channels (two is the default on '
                           'linux). On linux, the command is '
                           '"sudo /usr/sbin/virtualcan.sh start"\n')
                          .format(connect_timeout))

            rospy.sleep(0.5)

        def set_up_channel(chan):
            """
            Opens a Kvaser CAN channel
            """

            channel = canlib.openChannel(chan,
                                         flags=canlib.Open.ACCEPT_VIRTUAL,
                                         bitrate=canlib.canBITRATE_500K)
            channel.setBusOutputControl(canlib.Driver.NORMAL)
            channel.busOn()

            return channel

        # Open can channels, one for old and one for new frames
        self.channel_old = set_up_channel(0)
        self.channel_new = set_up_channel(1)

    def _read_can(self, channel, timeout=None):
        """
        Reads two messages from can at once. This is done because nodes publish two
        frames per cycle to the CAN bus: The first for steering angle and the
        second for speed
        """

        # If no timeout specified, use the default value of frame_timeout
        timeout = frame_timeout if timeout is None else timeout

        try:
            # Read two messages from CAN bus
            return (channel.read(timeout=timeout), channel.read(timeout=timeout))
        except canlib.canNoMsg:
            self.fail(('CAN frames were not detected in time. Set timeout '
                       'with the --connect-timeout option. The current value '
                       'is {}').format(connect_timeout))

    def _verify_frame(self, frame):
        """
        Checks for errors in a Kvaser CAN frame
        """

        # The frame is deemed OK if it has no error flag set
        self.assertFalse(frame.flags & canlib.MessageFlag.ERROR_FRAME)

    def _compare_frames(self, old, new):
        """
        Compares two Kvaser CAN frames to see if they contain the same data.

        Compares id, flags, dlc and data, but not timestamp
        """

        # Verify that new frame is error free, we are not testing the old code
        # here
        self._verify_frame(new)

        # Verify that the important fields of the frames are equal
        self.assertEqual(old.id, new.id, 'Frame id missmatch')
        self.assertEqual(old.flags, new.flags, 'Frame flags mismatch')
        self.assertEqual(old.dlc, new.dlc, 'Frame dlc missmatch')
        self.assertEqual(old.data, new.data, 'Frame data mismatch')

    def _compare_can_output(self, angle, speed):
        """
        Compares output of new and old code for a given angle and speed
        """

        # Publish requests for nodes to publish frames to CAN
        self.pub_old.publish(car_control(angle, speed))
        self.pub_new.publish(CarControl(angle, speed))

        # Read the frames from CAN
        angle_old, speed_old = self._read_can(self.channel_old)
        angle_new, speed_new = self._read_can(self.channel_new)

        # Make sure the new and old frames are the same
        self._compare_frames(angle_old, angle_new)
        self._compare_frames(speed_old, speed_new)

    # Unittests only runs class functions starting with test_
    def test_functions_same_as_2020(self):
        """
        Compare the output of new and old code given the same input
        """

        messages_to_send = 20  # Number of messages to perform tests on

        # Generate a bunch of messages and check that new and old code outputs
        # the same message to CAN
        for i in range(0, messages_to_send):
            angle = 80.0 * i / messages_to_send - 40
            speed = 10.0 * i / messages_to_send - 5

            self._compare_can_output(angle, speed)

    def _test_clamping_values(self, angle, speed):
        """
        Checks if the resulting message from the given angle and speed on the
        can bus have maxed out data values, meaning they haven't (probably)
        overflowed or anything like that.

        This method assumes angle and speed are large enough that they should
        get clamped
        """

        # Publish a request for frames frame to be put on the CAN bus
        self.pub_new.publish(CarControl(angle, speed))

        # Read the frames
        frame_angle, frame_speed = self._read_can(self.channel_new)

        # Verify that frames are error free
        self._verify_frame(frame_angle)
        self._verify_frame(frame_speed)

        # Check if values have been clamped
        self.assertEqual(frame_angle.data[0], 255)
        self.assertEqual(frame_speed.data[0], 255)

    def test_clamping_values(self):
        """
        Test if large positive and large negative values of speed and angle
        are clamped
        """

        # Test large positive values...
        self._test_clamping_values(1000, 1000)
        # ...and large negative values
        self._test_clamping_values(-1000, -1000)

    def tearDown(self):
        def tear_down_channel(channel):
            """
            Closes a Kvaser CAN channel
            """

            channel.busOff()
            channel.close()

        # Close channels
        tear_down_channel(self.channel_old)
        tear_down_channel(self.channel_new)


def main():
    global queue_size, connect_timeout, frame_timeout

    parser = argparse.ArgumentParser(description=DESCRIPTION)
    parser.add_argument('--queue-size', type=int, default=100,
                        help='Queue size of publishers')
    parser.add_argument('--connect-timeout', type=float, default=10,
                        help='Time limit for subscribers to pick up mesages')
    parser.add_argument('--frame-timeout', type=float, default=10,
                        help='Time limit for arrival of CAN frames')
    args, unknown = parser.parse_known_args()

    # Set some properties
    connect_timeout = args.connect_timeout
    queue_size = args.queue_size
    frame_timeout = int(round(args.frame_timeout * 1000))  # From (s) to (ms)

    # Start tests
    rostest.rosrun(PKG, NAME, TestCAN)


if __name__ == '__main__':
    main()
