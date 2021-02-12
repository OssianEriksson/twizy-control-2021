import rospy

from gnss.msg import GNSSLatLongHeight


def main(source):
    rospy.init_node('gnss')

    queue_size = rospy.get_param('~queue_size', 10)
    pub = rospy.Publisher('~gnss_llh', GNSSLatLongHeight, queue_size=queue_size)

    def publish_message(msg):
        pub.publish(msg)

    source.run(publish_message)
