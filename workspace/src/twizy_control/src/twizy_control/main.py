import rospy

from canlib import canlib, Frame

from twizy_msgs.msg import CarControl


def main():
    # Initialize ROS nodes
    rospy.init_node('control')

    # Read parameters from ROS parameter server
    tout = rospy.get_param('~timeout', 150)
    chan = rospy.get_param('~channel', 0)

    # Open Kvaser CAN channel
    channel = canlib.openChannel(chan, flags=canlib.Open.ACCEPT_VIRTUAL,
                                 bitrate=canlib.canBITRATE_500K)
    channel.setBusOutputControl(canlib.Driver.NORMAL)
    channel.busOn()

    def callback(msg):
        """
        Publishes control signals to CAN bus
        """

        # Prepare and write reference steering angle data
        angle_data = [int(min(round(abs(msg.angle) * 255.0 / 40), 255)),
                         0 if msg.angle < 0 else 255]
        channel.writeWait_raw(id_=150, msg=angle_data, dlc=8, timeout=tout)

        # Prepare and write reference speed
        speed_data = [int(min(round(abs(msg.speed) * 255.0 / 5), 255)),
                         0 if msg.speed > 0 else 255]
        channel.writeWait_raw(id_=154, msg=speed_data, dlc=8, timeout=tout)

    # Subscribe to topic where reference control values will be published
    rospy.Subscriber('twizy_control', CarControl, callback)

    # Wait for shutdown
    rospy.spin()
