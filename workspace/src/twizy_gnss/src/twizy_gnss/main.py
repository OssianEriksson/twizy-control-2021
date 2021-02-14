import rospy

from twizy_gnss.msg import GNSSLatLongHeight


def main(source):
    # Initialize ROS node
    rospy.init_node('gnss')

    # Get queue_size from the ROS parameter server. *tilde* (~) in front of a
    # name signals a private name, see http://wiki.ros.org/Names
    queue_size = rospy.get_param('~queue_size', 10)

    # Initialize publisher.
    pub = rospy.Publisher('~gnss_llh', GNSSLatLongHeight, queue_size=queue_size)

    def publish_message(msg):
        """
        Publish a message to the ~gnss_llh topic
        """

        pub.publish(msg)

    # Activate the source of GNSS data and provide it with a callback to send
    # back messages on
    source.run(publish_message)
