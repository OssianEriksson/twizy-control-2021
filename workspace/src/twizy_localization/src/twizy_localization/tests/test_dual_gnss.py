#!/usr/bin/env python

import rosunit
import unittest
import numpy as np

import rospy

from twizy_localization.dual_gnss import _reciever_positions_to_pose

PKG = 'twizy_localization'
NAME = 'test_dual_gnss'


def _pos_to_pose(pos, delta, origin, initial_delta):
    """
    Convert center position between GNSS recievers (pos) and relative position
    between recievers (delta) to geometry_msgs/PoseWithCovarianceStamped using
    _reciever_positions_to_pose from twizy_localization.main
    """

    # Convert from lists to numpy arrays
    pos = np.array(pos)
    delta = np.array(delta)
    origin = np.array(origin)
    initial_delta = np.array(initial_delta)

    return _reciever_positions_to_pose(pos, delta, origin, initial_delta,
                                       rospy.Time(0))


class TestMain(unittest.TestCase):

    def _quaternion_to_matrix(self, q):
        """
        Convert quaternion to rotation matrix. Sligthly edited code from
        https://pypi.org/project/transformations/
        """

        # Copyright (c) 2006, Christoph Gohlke
        # Copyright (c) 2006-2009, The Regents of the University of California
        # All rights reserved.
        #
        # Redistribution and use in source and binary forms, with or without
        # modification, are permitted provided that the following conditions
        # are met:
        #
        # * Redistributions of source code must retain the above copyright
        #   notice, this list of conditions and the following disclaimer.
        # * Redistributions in binary form must reproduce the above copyright
        #   notice, this list of conditions and the following disclaimer in the
        #   documentation and/or other materials provided with the
        #   distribution.
        # * Neither the name of the copyright holders nor the names of any
        #   contributors may be used to endorse or promote products derived
        #   from this software without specific prior written permission.
        #
        # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
        # "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
        # LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
        # A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
        # OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
        # SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
        # LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
        # DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
        # THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
        # (INCLUDING NEGLIGENCE OR OTHERWISE)  IN ANY WAY OUT OF THE USE OF
        # THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

        q = np.array([q.x, q.y, q.z, q.w])
        nq = np.dot(q, q)

        if nq < 1e-8:
            return np.identity(3)

        q *= np.sqrt(2.0 / nq)
        q = np.outer(q, q)

        return np.array((
            (1-q[1, 1]-q[2, 2],   q[0, 1]-q[2, 3],   q[0, 2]+q[1, 3]),
            (q[0, 1]+q[2, 3], 1-q[0, 0]-q[2, 2],   q[1, 2]-q[0, 3]),
            (q[0, 2]-q[1, 3],   q[1, 2]+q[0, 3], 1-q[0, 0]-q[1, 1]),
        ))

    def _check_pose(self, pose, pos, rot):
        """
        Checks if a pose is such that its position aligns with pos and its
        orientation is such that it transforms the vector [1, 0, 0] to rot
        """

        p = pose.pose.pose.position
        p = [p.x, p.y, p.z]

        # Check if positions are equal
        self.assertTrue(all(np.isclose(p, pos)), 'Pos {} != {}'.format(p, pos))

        # Convert quaternion to rotation matrix
        R = self._quaternion_to_matrix(pose.pose.pose.orientation)
        # Apply orientation transformation to the vector [1, 0, 0]
        r = np.dot(R, [1, 0, 0])

        # Check if the resulting vector is close to the expected
        self.assertTrue(all(np.isclose(r, rot)), 'Rot {} != {}'.format(r, rot))

    def test_position(self):
        """
        Test some different translations
        """

        pose = _pos_to_pose([1, 2, 3], [1, 0, 0], [0, 0, 0], [1, 0, 0])
        self._check_pose(pose, [1, 2, 3], [1, 0, 0])

        pose = _pos_to_pose([1, 2, 3], [1, 0, 0], [4, 3, 2], [1, 0, 0])
        self._check_pose(pose, [-3, -1, 1], [1, 0, 0])

    def test_orientation_coordinate_axis(self):
        """
        Test rotation with final position aligned to coordinate axis
        """

        # No rotation
        pose = _pos_to_pose([0, 0, 0], [1, 0, 0], [0, 0, 0], [1, 0, 0])
        self._check_pose(pose, [0, 0, 0], [1, 0, 0])

        # 90 degrees around z
        pose = _pos_to_pose([0, 0, 0], [0, 1, 0], [0, 0, 0], [1, 0, 0])
        self._check_pose(pose, [0, 0, 0], [0, 1, 0])

        # 180 degrees around z
        pose = _pos_to_pose([0, 0, 0], [-1, 0, 0], [0, 0, 0], [1, 0, 0])
        self._check_pose(pose, [0, 0, 0], [-1, 0, 0])

        # 270 degrees around z
        pose = _pos_to_pose([0, 0, 0], [0, -1, 0], [0, 0, 0], [1, 0, 0])
        self._check_pose(pose, [0, 0, 0], [0, -1, 0])

        # 90 degrees around y
        pose = _pos_to_pose([0, 0, 0], [0, 0, -1], [0, 0, 0], [1, 0, 0])
        self._check_pose(pose, [0, 0, 0], [0, 0, -1])

        # 270 degrees around y
        pose = _pos_to_pose([0, 0, 0], [0, 0, 1], [0, 0, 0], [1, 0, 0])
        self._check_pose(pose, [0, 0, 0], [0, 0, 1])


if __name__ == '__main__':
    rosunit.unitrun(PKG, NAME, TestMain)
