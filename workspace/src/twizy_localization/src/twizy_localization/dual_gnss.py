import rospy

import numpy as np
import utm

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped


def _to_enu(msg):
    """
    Convert NavSatFix to East North Up coordinate system using UTM
    """

    utm_point = utm.from_latlon(msg.latitude, msg.longitude)

    return np.array([utm_point[0], utm_point[1], msg.altitude])


def _normalize(v):
    """
    Normalize a vector
    """

    return np.divide(v, np.sqrt(np.sum(v**2)))


def _reciever_positions_to_pose(pos, delta, origin, initial_delta, stamp=None,
                                variance=[0, 0, 0], frame_id='odom'):
    """
    Convert the center and relative position of two GNSS recievers into a
    PoseWithCovarianceStamped message with timestamp stamp

    variance is the variance of coodrinates x, y, z in the vector delta. We
    discard other covariance values for now, because the math is hard. On top
    of that the other covariance values are always zero with the GNSS recievers
    we are currently using, so right now that extra math would be completely
    useless as well...
    """

    # Default timestamp to current time
    stamp = rospy.Time.now() if stamp is None else stamp

    # There are more efficient ways of doing this, but im tired and want this
    # working so here goes:

    # Relative position
    pos = np.subtract(pos, origin)

    # Axis of rotation
    axis = np.cross(initial_delta, delta)

    # Sin and cos of angle around axis of rotation
    sinus = np.sqrt(np.sum(axis**2))
    cosinus = np.dot(initial_delta, delta)

    # Angle of rotation
    angle = np.arctan2(sinus, cosinus)

    # Normalize the axis, but dont bother if the vector is close to singular.
    # If it is close to singular that means we rotated either close to 0 or 180
    # degreens. In these cases it doesn't matter what axis we use as long as it
    # is ortogonal to initial_delta (or delta as initial_delta and delta will)
    # be almost parallel
    if not np.isclose(sinus, 0):
        # The sinus value is simultaneously sin(angle) and the length of the
        # axis vector
        axis = np.divide(axis, sinus)
    else:
        axis = np.cross(initial_delta, [1, 0, 0])
        length = np.sqrt(np.sum(axis**2))

        if not np.isclose(sinus, 0):
            axis = np.divide(axis, length)
        else:
            axis = _normalize(np.cross(initial_delta, [0, 1, 0]))

    # Needed for calculating quaternion from axle of rotation and angle
    s2 = np.sin(angle / 2)
    c2 = np.cos(angle / 2)


    # Most elements in the covariance matrix is zeoro since we discarded all
    # but the diagonal GNSS covariance values
    covariance = [0.0] * 36

    # Set position covariance. We're dividing by 4 since the position is the
    # mean of two measurements and 2^2=4, basic probability theory people...
    covariance[0] = variance[0] / 4.0
    covariance[7] = variance[1] / 4.0
    covariance[14] = variance[2] / 4.0

    # So this next bit is basically an educated guess at what the variances
    # might be. There is some reasoning behind them which I can't be bothered
    # to explain (ha), but if you don't know what you're doing I say leave the
    # following as it is and don't worry about it. I'm estimating the chance of
    # this working close to intended at around 50%. If you DO know what you're
    # doing, please check these calculations, since as I said, I made most of
    # it up... (Also you can try integrating non-diagonal covariance elements
    # into the calculations if you are REALLY desparate for a probability
    # theory fix)
    cov_coeff = np.divide(np.abs(delta), np.sqrt(np.sum(delta**2)))
    covariance[21] = variance[1] * cov_coeff[2] + variance[2] * cov_coeff[1]
    covariance[28] = variance[2] * cov_coeff[0] + variance[0] * cov_coeff[2]
    covariance[35] = variance[0] * cov_coeff[1] + variance[1] * cov_coeff[0]

    # Create PoseWithCovarianceStamped message
    pose = PoseWithCovarianceStamped()
    pose.header.stamp = stamp
    pose.header.frame_id = frame_id
    pose.pose.covariance = covariance
    pose.pose.pose.orientation.x = axis[0] * s2
    pose.pose.pose.orientation.y = axis[1] * s2
    pose.pose.pose.orientation.z = axis[2] * s2
    pose.pose.pose.orientation.w = c2
    pose.pose.pose.position.x = pos[0]
    pose.pose.pose.position.y = pos[1]
    pose.pose.pose.position.z = pos[2]

    return pose


class GNSSData:
    def __init__(self):
        self.position = None
        self.variance = None


class _DifferentialGNSS:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('dual_gnss')

        # Initialize storage for recieved GNSS data
        self.data = {'left': GNSSData(), 'right': GNSSData()}

        self.origin = None  # Origin of coordinate system

        # Create subscribers
        rospy.Subscriber('/gnss/left/navsatfix_best_fix', NavSatFix,
                         callback=self.cb_piksi, callback_args='left',
                         queue_size=1)
        rospy.Subscriber('/gnss/right/navsatfix_best_fix', NavSatFix,
                         callback=self.cb_piksi, callback_args='right',
                         queue_size=1)

        # Create publishers
        self.pub_pose = rospy.Publisher(
            'pose', PoseWithCovarianceStamped, queue_size=10)

        # Wait until shutdown
        rospy.spin()

    def cb_piksi(self, msg, side):
        """
        Callback for ROS subscribers to Piksi GNSS data
        """

        # Store conversion to ENU coordinates
        self.data[side].position = _to_enu(msg)

        # Store variance (discard other covariances for now...)
        self.data[side].variance = msg.position_covariance[::4]

        l = self.data['left'].position   # Position of left GNSS reciever
        r = self.data['right'].position  # Position of right GNSS reciever

        # Only continue if we have recieved messages from both GNSS antennas
        if l is None or r is None:
            return

        pos = np.divide(np.add(l, r), 2)  # Center of the two GNSS recievers
        delta = _normalize(np.subtract(l, r))

        # Set the origin to the position of the Twizy at startup
        if self.origin is None:
            self.origin = pos
            self.initial_delta = delta

        # Calculate delta variance
        variance = np.add(self.data['left'].variance,
                          self.data['right'].variance)

        # Calculate pose from position and relative position of GNSS recievers
        pose = _reciever_positions_to_pose(pos, delta, self.origin,
                                           self.initial_delta,
                                           msg.header.stamp, variance)

        # Publish the pose
        self.pub_pose.publish(pose)


def main():
    _DifferentialGNSS()
