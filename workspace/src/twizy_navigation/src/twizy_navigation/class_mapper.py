import rospy

import threading

from sensor_msgs.msg import Image
from twizy_msgs.msg import BoundingBoxes


def main():
    rospy.init_node('class_mapper')

    def cb_image(img):
        pass

    def cb_bbs(bbs):
        pass

    rospy.Subscriber('bounding_boxes', BoundingBoxes,
                     callback=cb_bbs, queue_size=1)
    rospy.Subscriber('depth_image', Image, callback=cb_image, queue_size=1)

    rospy.spin()
