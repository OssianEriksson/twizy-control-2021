import rospy

from gnss.msg import GNSSLatLongHeight

from sbp.client.drivers.network_drivers import TCPDriver
from sbp.client import Handler, Framer
from sbp.navigation import SBP_MSG_POS_LLH


class TCPSource:

    def _parse_sbp(self, msg):
        return GNSSLatLongHeight(latitude=msg.lat,
                                 longitude=msg.lon,
                                 height=msg.height,
                                 h_accuracy=msg.h_accuracy / 1000.0,  # mm to m
                                 v_accuracy=msg.v_accuracy / 1000.0,  # mm to m
                                 n_satellites=msg.n_sats)

    def run(self, callback):
        host = rospy.get_param('~host', '127.0.0.1')
        port = rospy.get_param('~port', 55555)
        
        with TCPDriver(host, port) as driver:
            with Handler(Framer(driver.read, None, verbose=True)) as source:
                try:
                    for msg, metadata in source.filter(SBP_MSG_POS_LLH):
                        callback(self._parse_sbp(msg))

                        if rospy.is_shutdown():
                            break
                except KeyboardInterrupt:
                    pass
