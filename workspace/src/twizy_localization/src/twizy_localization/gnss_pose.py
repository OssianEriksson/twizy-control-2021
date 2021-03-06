import tf2_ros
import rospy

from geometry_msgs.msg import Point, PointStamped, Quaternion, PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix

from tf2_geometry_msgs import do_transform_point
import utm
import numpy as np
from numpy import cos, sin, pi, sqrt
from tf.transformations import quaternion_from_matrix


def align_vectors(a, b, weights):
    """
    Largely works like newer versions of scipy's scipy.stats.align_vectors():
    https://docs.scipy.org/doc/scipy/reference/generated/
    scipy.spatial.transform.Rotation.align_vectors.html
    The code in this function is mostly copied frome there, with
    modififications

    Finds optimal rotation matrix and sensativity matrix for transform between
    the point sets a and b
    """

    # Copyright (c) 2001-2002 Enthought, Inc.  2003-2019, SciPy Developers.
    # All rights reserved.
    #
    # Redistribution and use in source and binary forms, with or without
    # modification, are permitted provided that the following conditions
    # are met:
    #
    # 1. Redistributions of source code must retain the above copyright
    #    notice, this list of conditions and the following disclaimer.
    #
    # 2. Redistributions in binary form must reproduce the above
    #    copyright notice, this list of conditions and the following
    #    disclaimer in the documentation and/or other materials provided
    #    with the distribution.
    #
    # 3. Neither the name of the copyright holder nor the names of its
    #    contributors may be used to endorse or promote products derived
    #    from this software without specific prior written permission.
    #
    # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    # "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    # LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
    # A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    # OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    # SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    # LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    # DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    # THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    a = np.asarray(a)
    b = np.asarray(b)
    weights = np.asarray(weights)

    B = np.einsum('ji,jk->ik', weights[:, None]*a, b)
    u, s, vh = np.linalg.svd(B)

    # Correct improper rotation if necessary (as in Kabsch algorithm)
    if np.linalg.det(np.matmul(u, vh)) < 0:
        s[-1] = -s[-1]
        u[:, -1] = -u[:, -1]

    C = np.identity(4)
    C[:3, :3] = np.dot(u, vh)

    if s[1] + s[2] < 1e-16*s[0]:
        rospy.logwarn('Optimal rotation is not uniquely or poorly defined for '
                      'the given sets of vectors.')

    zeta = (s[0] + s[1])*(s[1] + s[2])*(s[2] + s[0])
    kappa = s[0]*s[1] + s[1]*s[2] + s[2]*s[0]
    with np.errstate(divide='ignore', invalid='ignore'):
        sensitivity = np.mean(weights)/zeta*(kappa*np.eye(3) + np.dot(B, B.T))

    return C, sensitivity


def rotation_from_point_sets(r_local, r_global, r_global_variance):
    """
    Returns rotation matrix and covariance matrix for rotation from the point
    set r_local to the point set r_global (where points have variance
    r_global_variance). Each entry in r_local must correspond to the entry at
    the same index in r_global. If only two points are present in each point
    set, the rotation returned will have no component around the axis between
    the two points
    """

    # Subtract centeroid from positions. This is neccessary for the Kabsch
    # algoritm which align_vectors() uses to estimate rotations later on
    r_local -= np.mean(r_local, axis=0)
    r_global -= np.mean(r_global, axis=0)

    # Rotation cannot be judged from only two position measurements. In
    # order to support robots with only two sources of NavSatFix data, we
    # insert a third (constructed) position measurement choosen such that
    # the resulting rotation computation will give a pitch value of 0
    if r_local.shape[0] == 2:
        # Compute vector parallel to ground plane in local frame
        r_local = np.vstack([r_local,
                             np.cross(r_local[0] - r_local[1], [0, 0, 1])])
        r_local[2] /= np.linalg.norm(r_local[2])

        # Compute vector parallel to ground plane in global frame
        r_global = np.vstack([r_global,
                              np.cross(r_global[0] - r_global[1], [0, 0, 1])])
        norm2_r_global = np.sum(r_global[2]**2)
        r_global[2] /= sqrt(norm2_r_global)

        # Set variance for constructed position measurement
        r_g_var = (r_global_variance[0] + r_global_variance[1])/norm2_r_global
        r_global_variance = np.vstack([r_global_variance, r_g_var])

        # Improvised variance adjustment to counteract the addition of one
        # extra data point
        r_global_variance *= 3.0 / 2.0

    # Optimal weights are inversely proportional to variances of measurements
    weights = 1.0 / np.maximum(1e-8, np.max(r_global_variance, axis=1))

    # Estimate rotation matrix and sensitivity matrix
    C, sensitivity = align_vectors(r_global, r_local, weights)

    # By multiplying sensitivity with harmonic mean of variances we get a
    # covariance matrix.
    covariance = sensitivity / np.mean(weights)

    return C, covariance


def main():
    # Initialize ROS node
    rospy.init_node('gnss_pose')

    # Read parameters from ROS parameter server
    center_link = rospy.get_param('~center_link')
    frequency = rospy.get_param('~frequency', 30)
    timeout = rospy.get_param('~timeout', 0.5)
    map_frame = rospy.get_param('~map_frame', 'map')
    differential = rospy.get_param('~differential', False)

    timeout = rospy.Duration(timeout)

    # Initialize tf buffer for buffering transforms between frames
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    navsatfixes = {}  # dict of NavSatFix messages to include in computation

    def cb_navsatfix(navsatfix):
        """
        Callback for NavSatFix messages. Adds the incoming message to the
        navsatfixes dict with its frame_id as key
        """

        navsatfixes[navsatfix.header.frame_id] = navsatfix

    # Subscribe to incoming NavSatFix messages
    rospy.Subscriber('/gnss/fix', NavSatFix,
                     callback=cb_navsatfix, queue_size=1)

    # Publisher for estimated pose from GNSS data
    pub_pose = rospy.Publisher(
        '/gnss/pose', PoseWithCovarianceStamped, queue_size=1)

    # Initialize pose which is to be published later
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = map_frame

    initial_position = None  # Holds initial position of robot

    rate = rospy.Rate(frequency)  # Rate to update at

    while not rospy.is_shutdown():
        rate.sleep()

        # Remove NavSatFix messages from the buffer if they are too old
        for frame_id, navsatfix in navsatfixes.items():
            if navsatfix.header.stamp + timeout < rospy.Time.now():
                navsatfixes.pop(frame_id)

        n_navsats = len(navsatfixes)

        # We cannot calculate rotation from only one (or zero) point(s)
        if n_navsats < 2:
            continue

        stamp = rospy.Time.now()

        # Prepare points in local coordinates (relative to the robot's
        # center_link frame which should lie at the mean position of the GNSS
        # recievers position on the robot) and in global coordinates (ENU) for
        # each GNSS reciever. Also store variances for the global position (the
        # local position is assumed to have no variance)
        r_local, r_global = np.zeros((n_navsats, 3)), np.zeros((n_navsats, 3))
        r_global_variance = np.zeros((n_navsats, 3))
        for i, (frame_id, navsatfix) in enumerate(navsatfixes.items()):
            # Find position of GNSS reciever in local (center_link) frame
            r_local_vec = tfBuffer.lookup_transform(
                target_frame=center_link,
                source_frame=frame_id,
                time=navsatfix.header.stamp,
                timeout=rospy.Duration(1.0)
            ).transform.translation
            r_local[i] = [r_local_vec.x, r_local_vec.y, r_local_vec.z]

            # Convert NavSatFix's latitude, longitude, altitude to cartesian
            # coordinates (UTM/ENU)
            r_global_utm = utm.from_latlon(navsatfix.latitude,
                                           navsatfix.longitude)
            r_global[i] = [r_global_utm[0],
                           r_global_utm[1], navsatfix.altitude]

            # Discard all the cross covariances of the NavSatFix message as the
            # math gets a lot easier this way :)
            r_global_variance[i] = navsatfix.position_covariance[::4]

        # Estimated pose position is mean of all GNSS recievers positions
        position = np.mean(r_global, axis=0)
        position_covariance = np.sum(r_global_variance, axis=0) / n_navsats**2

        # If in differential mode, we want to subtract the initial position
        # from each position measurement so the robot always starts at position
        # (0, 0, 0)
        if initial_position is None:
            initial_position = position
        if differential:
            position = position - initial_position

        # Estimate rotation matrix and covariance matrix
        C, rotation_covariance = rotation_from_point_sets(r_local, r_global,
                                                          r_global_variance)

        # Convert rotation matrix to quaternion
        orientation = quaternion_from_matrix(C)
        # Use the largest possible covariance (upper bound using matrix norm)
        # as covariance around all three axis of rotation
        orientation_covariance = np.linalg.norm(rotation_covariance)

        # Construct pose message
        pose.header.stamp = stamp
        pose.pose.covariance = [0.0]*36
        pose.pose.covariance[:15:7] = position_covariance
        pose.pose.covariance[21::7] = [orientation_covariance] * 3
        pose.pose.pose.position = Point(*position)
        pose.pose.pose.orientation = Quaternion(*orientation)

        # Publish pose message
        pub_pose.publish(pose)
