import rospy

from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import PointStamped
from rospy.msg import AnyMsg

import numpy as np
from math import isnan
import utm
import twizy_webots.util as util


def main():
    # Initialize ROS node. There might be multiple versions of this node
    # running at once with the same name, anonymous=True allows this
    rospy.init_node('gps', anonymous=True)

    # Read parameters from ROS parameter server
    device = rospy.get_param('~device')
    frame_id = rospy.get_param('~frame_id', None)
    ups = rospy.get_param('ups', 30)
    status = rospy.get_param('status', NavSatStatus.STATUS_FIX)
    service = rospy.get_param('service', NavSatStatus.SERVICE_GALILEO)
    position_covariance = rospy.get_param('position_covariance', [0, 0, 0,
                                                                  0, 0, 0,
                                                                  0, 0, 0])
    position_covariance_type = rospy.get_param(
        'position_covariance_type', NavSatFix.COVARIANCE_TYPE_UNKNOWN)
    gps_reference = rospy.get_param('/gps_reference', [0, 0, 0])

    # Get webots model name
    model_name = util.model_name()

    # Attempt to enable the device
    if not util.enable(model_name, device, ups):
        rospy.logfatal('Unable to enable device {}'.format(device))
        return

    # Prepare offsets
    x_off, y_off, zone_num, zone_letter = utm.from_latlon(gps_reference[0],
                                                          gps_reference[1])

    # Prepare NavSatFix message
    navsatfix = NavSatFix()
    navsatfix.status.status = status
    navsatfix.status.service = service
    navsatfix.position_covariance = position_covariance
    navsatfix.position_covariance_type = position_covariance_type

    # Numpy array for noise generation
    mean = np.array([x_off, y_off, gps_reference[2]])
    cov = np.reshape(position_covariance, (3, 3))

    # Initialize publisher for converted messages
    pub_navsatfix = rospy.Publisher('/navsatfix', NavSatFix, queue_size=1)

    def cb_point(p):
        """
        Callback for converting a PointStamped p to NavSatFix and
        publishing it on pub_navsatfix
        """

        # For the first few frames of the simulation, NaN may be published by
        # the webots ROS controller. Just discard those values
        if isnan(p.point.x) or isnan(p.point.y):
            return

        # Copy header from incoming message
        navsatfix.header = p.header

        # Update frame_id
        navsatfix.header.frame_id = util.frame_id(p.header.frame_id, frame_id)

        # Generate noise, mean contains offset used to define the model's
        # starting position
        noise = np.random.multivariate_normal(mean, cov, size=None)

        # Convert from NUE to ENU, add noise and offset in cartesian coodinates
        x = p.point.z + noise[0]
        y = p.point.x + noise[1]
        navsatfix.altitude = p.point.y + noise[2]

        # Clamp x and y coordinates to the maximum values allowed by utm


        # Convert back to latitude, longitude
        try:
            # This function call will throw a OutOfRangeError if (x, y) are
            # outside of the supported ranges
            lat, lon = utm.to_latlon(x, y, zone_num, zone_letter)
        except utm.OutOfRangeError as e:
            # If the above fail, redo the calculation but without throwing
            # the error ...
            lat, lon = utm.to_latlon(x, y, zone_num, zone_letter, strict=False)

            # ... then log what happened
            rospy.logerr('{} (coordinates were [{}; {}])'.format(e, x, y))

        navsatfix.latitude = lat
        navsatfix.longitude = lon

        # Publish converted message
        pub_navsatfix.publish(navsatfix)

    # Subscribe to PointStamped topic published by webots ROS controller
    values_name = '/{}/{}/values'.format(model_name, device)
    rospy.Subscriber(values_name, PointStamped,
                     callback=cb_point, queue_size=1)

    # Wait for shutdown
    rospy.spin()
