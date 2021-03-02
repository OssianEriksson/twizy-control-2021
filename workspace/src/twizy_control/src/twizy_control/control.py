import math

import rospy

from canlib import canlib, Frame

from twizy_msgs.msg import CarControl


def main():
    # Initialize ROS node
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

        angle = msg.angle * 180 / math.pi  # rad to degrees
        speed = msg.speed * 3.6            # m/s to km/h

        # Prepare and write reference steering angle data
        angle_data = [int(min(round(abs(angle) * 255.0 / 40), 255)),
                      0 if angle < 0 else 255]
        channel.writeWait_raw(id_=150, msg=angle_data, dlc=8, timeout=tout)

        # Prepare and write reference speed
        speed_data = [int(min(round(abs(speed) * 255.0 / 5), 255)),
                      0 if speed > 0 else 255]
        channel.writeWait_raw(id_=154, msg=speed_data, dlc=8, timeout=tout)

    # Subscribe to topic where reference control values will be published
    rospy.Subscriber('twizy_control', CarControl, callback, queue_size=1)

    # Wait for shutdown
    rospy.spin()
