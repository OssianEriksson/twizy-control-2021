#!/usr/bin/env python

import rosunit
import unittest
import numpy as np
from numpy import pi

import rospy

from tf.transformations import euler_matrix, euler_from_matrix

from twizy_localization.gnss_pose import rotation_from_point_sets

PKG = 'twizy_localization'
NAME = 'test_gnss_pose'


class TestMain(unittest.TestCase):
    def _pts2rot(self, r_local, r_global, r_global_varaiance=None):
        """
        Estimate orientation from arrays of GNSS reciever locations - r_local
        in local frame and r_global in global frame. If no variances are
        specified, ones are used.
        """

        # Convert from lists to numpy arrays
        r_local = np.array(r_local)
        r_global = np.array(r_global)

        if r_global_varaiance is None:
            r_global_varaiance = np.ones(r_local.shape)
        else:
            r_global_varaiance = np.array(r_global_varaiance)

        return rotation_from_point_sets(r_local, r_global, r_global_varaiance)

    def _rpy_bounce2(self, r, p, y, axis=[0, 1, 0]):
        """
        Creates data set with two positions sensors with relative position
        given by axis. Rotates the points by the provided roll (r), pitch (p)
        and yaw (2) and returns what gnss_pose.rotation_from_point_sets thought
        what the values for roll pitch and yaw were
        """

        R = euler_matrix(r, p, y)

        axis = [axis[0], axis[1], axis[2], 1.0]
        r_local = np.matrix([[0, 0, 0, 1.0], axis])
        r_global = np.matmul(R, r_local.T).T

        C, cov = self._pts2rot(r_local[:, :3], r_global[:, :3])

        return self._rpy_mod(*euler_from_matrix(C))

    def _rpy_mod(self, r, p, y):
        """
        Loops roll pitch yaw values back on themselves if they are too large or
        too small
        """

        return (r + pi)%(2*pi) - pi, (p + pi/2)%pi - pi/2, (y + pi)%(2*pi) - pi

    def test_no_pitch(self):
        """
        Makes sure no pitch is returned from
        gnss_pose.rotation_from_point_sets() when only two data points along
        the y-axis are provided
        """

        def test(r, p, y):
            r_, p_, y_ = self._rpy_bounce2(r, p, y, [0, 1, 0])

            self.assertAlmostEquals(p_, 0, msg='Pitch detected for rpy='
                                    '"{} {} {}"'.format(r, p, y))

        test(0, 0, 0)
        
        for r in np.linspace(-2*pi, 2*pi, 5):
            for p in np.linspace(-2*pi, 2*pi, 5):
                for y in np.linspace(-2*pi, 2*pi, 5):
                    test(r, p, y)

    def test_pitch_roll(self):
        """
        When only using two position sensors, test that values for roll and
        yaw are always correct
        """

        def test(r, y):
            r, p, y = self._rpy_mod(r, 0, y)
            r_, p_, y_ = self._rpy_bounce2(r, p, y, [0, 1, 0])

            self.assertAlmostEquals(r_ % pi, r % pi, msg='Mismatched roll for '
                                    'rpy="{} {} {}"'.format(r, 0, y))
            self.assertAlmostEquals(y_ % (2*pi), y % (2*pi), msg='Mismatched '
                                    'yaw for rpy="{} {} {}"'.format(r, 0, y))

        test(0, 0)

        for r in np.linspace(-pi/2, pi/2, 10):
            for y in np.linspace(-pi, pi, 10):
                test(r, y)


if __name__ == '__main__':
    rosunit.unitrun(PKG, NAME, TestMain)
