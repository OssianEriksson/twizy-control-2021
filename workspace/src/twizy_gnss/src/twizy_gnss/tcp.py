import rospy

from twizy_gnss.msg import GNSSLatLongHeight

from sbp.client.drivers.network_drivers import TCPDriver
from sbp.client import Handler, Framer
from sbp.navigation import SBP_MSG_POS_LLH


class TCPSource:
    """
    Source of GNSS position data based on listening for SBP messages on a TCP
    socket
    """

    def _parse_sbp(self, msg):
        """
        Convert SBP representation of position data to ROS message
        """

        return GNSSLatLongHeight(latitude=msg.lat,
                                 longitude=msg.lon,
                                 height=msg.height,
                                 h_accuracy=msg.h_accuracy / 1000.0,  # mm to m
                                 v_accuracy=msg.v_accuracy / 1000.0,  # mm to m
                                 n_satellites=msg.n_sats)

    def run(self, callback):
        """
        Start listening for GNSS data on a TCP socket with a ROS-param-provided
        host and port
        """

        # Retrive parameters from the ROS parameter server
        host = rospy.get_param('~host', '127.0.0.1')
        port = rospy.get_param('~port', 55555)
        
        # Almost copy-pasted from examples of the sbp package
        with TCPDriver(host, port) as driver:
            with Handler(Framer(driver.read, None, verbose=True)) as source:
                try:
                    # Filter on SBP_MSG_POS_LLH-messages (LLH stands for 
                    # Longitude Latitude Height)
                    for msg, metadata in source.filter(SBP_MSG_POS_LLH):
                        # Send the message back for publishing
                        callback(self._parse_sbp(msg))

                        if rospy.is_shutdown():
                            break
                except KeyboardInterrupt:
                    pass
