import rospy

from sensor_msgs.msg import NavSatFix, NavSatStatus

from sbp.client.drivers.network_drivers import TCPDriver
from sbp.client import Handler, Framer
from sbp.navigation import SBP_MSG_POS_LLH


def _publish_navsatfix(pub, msg, frame_id):
    """
    Converts a MSG_POS_LLH SBP message to a NavSatFix message and publishes
    it on the provided publisher pub. Most of this code is borrowed from 
    https://github.com/ethz-asl/ethz_piksi_ros/tree/master/piksi_multi_rtk_ros
    """

    # Copyright (c) 2017, Autonomous Systems Lab & Robotic Systems Lab,
    # ETH Zurich
    # All rights reserved.

    # Redistribution and use in source and binary forms, with or without
    # modification, are permitted provided that the following conditions are
    # met:

    # 1. Redistributions of source code must retain the above copyright
    # notice, this list of conditions and the following disclaimer.

    # 2. Redistributions in binary form must reproduce the above copyright
    # notice, this list of conditions and the following disclaimer in the
    # documentation and/or other materials provided with the distribution.

    # 3. Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
    # names of its contributors may be used to endorse or promote products
    # derived from this software without specific prior written permission.

    # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
    # IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
    # THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
    # PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
    # CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
    # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
    # PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
    # PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
    # LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
    # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    # SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    # Convert standard deviation in mm to variance in m^2
    h_var = (msg.h_accuracy / 1000)**2
    v_var = (msg.h_accuracy / 1000)**2

    # Lower 3 bits define fix mode.
    fix_mode = msg.flags & 0b111

    if fix_mode == 0:  # Invalid mode
        navsat_status = NavSatStatus.STATUS_NO_FIX
    elif fix_mode == 1:
        navsat_status = NavSatStatus.STATUS_FIX
    elif fix_mode == 2:  # Not implemented
        navsat_status = NavSatStatus.STATUS_NO_FIX
    elif fix_mode == 3:
        navsat_status = NavSatStatus.STATUS_GBAS_FIX
    elif fix_mode == 4:
        navsat_status = NavSatStatus.STATUS_GBAS_FIX
    elif fix_mode == 5:  # Not implemented
        navsat_status = NavSatStatus.STATUS_NO_FIX
    elif fix_mode == 6:
        navsat_status = NavSatStatus.STATUS_SBAS_FIX
    else:  # Undefined modes
        navsat_status = NavSatStatus.STATUS_NO_FIX

    # Again, (almost) all copied from
    # http://github.com/ethz-asl/ethz_piksi_ros/tree/master/piksi_multi_rtk_ros
    navsatfix = NavSatFix()
    navsatfix.header.stamp = rospy.Time.now()
    navsatfix.header.frame_id = frame_id
    navsatfix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
    navsatfix.status.service = NavSatStatus.SERVICE_GPS
    navsatfix.latitude = msg.lat
    navsatfix.longitude = msg.lon
    navsatfix.altitude = msg.height
    navsatfix.status.status = navsat_status
    navsatfix.position_covariance = [h_var, 0, 0,  # This I came up with
                                     0, h_var, 0,  # myself. ethz_piksi_ros
                                     0, 0, v_var]  # uses a constant cov-matrix

    # Publish to ROS topic
    pub.publish(navsatfix)


def main():
    # Initialize ROS node. We imitate node name present in
    # http://github.com/ethz-asl/ethz_piksi_ros/tree/master/piksi_multi_rtk_ros
    rospy.init_node('piksi')

    # Initialize publisher. We imitate publisher names present in
    # http://github.com/ethz-asl/ethz_piksi_ros/tree/master/piksi_multi_rtk_ros
    pub = rospy.Publisher('~navsatfix_best_fix', NavSatFix, queue_size=10)

    # Retrive parameters from the ROS parameter serve
    # We imitate parameter names present in
    # http://github.com/ethz-asl/ethz_piksi_ros/tree/master/piksi_multi_rtk_ros
    host = rospy.get_param('~tcp_addr', '192.168.0.222')
    port = rospy.get_param('~tcp_port', 55555)
    frame_id = rospy.get_param('~navsatfix_frame_id', 'gps')

    # Start listening for GNSS data on a TCP socket with a ROS-param-provided
    # host and port. This code is almost copy-pasted from examples of the sbp
    # package
    with TCPDriver(host, port) as driver:
        with Handler(Framer(driver.read, None, verbose=True)) as source:
            try:
                # Filter on SBP_MSG_POS_LLH-messages (LLH stands for
                # Longitude Latitude Height)
                for msg, metadata in source.filter(SBP_MSG_POS_LLH):
                    _publish_navsatfix(pub, msg, frame_id)

                    if rospy.is_shutdown():
                        break
            except KeyboardInterrupt:
                pass
