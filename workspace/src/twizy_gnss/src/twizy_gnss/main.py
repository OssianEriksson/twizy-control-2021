import rospy

import numpy as np
import utm

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped


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
                                frame_id='/world'):
    """
    Convert the center and relative position of two GNSS recievers into a
    PoseStamped message with timestamp stamp
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
    angle = np.arctan2(cosinus, sinus)

    # Normalize the axis, but dont bother if the vector is close to singular,
    # then the normalization wont really matter anyway
    if not np.isclose(sinus, 0):
        # The sinus value is simultaneously sin(angle) and the length of the
        # axis vector
        axis = np.divide(axis, sinus)

    # Needed for calculating quaternion from axle of rotation and angle
    s2 = np.sin(angle / 2)
    c2 = np.cos(angle / 2)

    # Create PoseStamped message
    pose = PoseStamped()
    pose.header.stamp = stamp
    pose.header.frame_id = frame_id
    pose.pose.orientation.x = axis[0] * s2
    pose.pose.orientation.y = axis[1] * s2
    pose.pose.orientation.z = axis[2] * s2
    pose.pose.orientation.w = c2
    pose.pose.position.x = pos[0]
    pose.pose.position.y = pos[1]
    pose.pose.position.z = pos[2]

    return pose


class _DifferentialGNSS:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('twizy_gnss')

        self.positions = {'left': None, 'right': None}  # Positions of left and
        # right GNSS reciever

        self.origin = None  # Origin of coordinate system

        # Create subscribers
        rospy.Subscriber('/gnss/left/navsatfix_best_fix', NavSatFix,
                         callback=self.cb_piksi, callback_args='left')
        rospy.Subscriber('/gnss/right/navsatfix_best_fix', NavSatFix,
                         callback=self.cb_piksi, callback_args='right')

        # Create publishers
        self.pub_pose = rospy.Publisher(
            'twizy_pose', PoseStamped, queue_size=10)

        # Wait until shutdown
        rospy.spin()

    def cb_piksi(self, msg, side):
        """
        Callback for ROS subscribers to Piksi GNSS data
        """

        # Convert to ENU coordinates
        self.positions[side] = _to_enu(msg)

        l = self.positions['left']   # Position of left GNSS reciever
        r = self.positions['right']  # Position of right GNSS reciever

        # Only continue if we have recieved messages from both GNSS antennas
        if l is None or r is None:
            return

        pos = np.divide(np.add(l, r), 2)  # Center of the two GNSS recievers
        delta = _normalize(np.subtract(l, r))

        # Set the origin to the position of the Twizy at startup
        if self.origin is None:
            self.origin = pos
            self.initial_delta = delta

        # Calculate pose from position and relative position of GNSS recievers
        pose = _reciever_positions_to_pose(
            pos, delta, self.origin, self.initial_delta, msg.header.stamp)

        # Publish the pose
        self.pub_pose.publish(pose)


def main():
    _DifferentialGNSS()
