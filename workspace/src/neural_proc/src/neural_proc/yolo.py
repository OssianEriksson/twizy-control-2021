import rospy

from sensor_msgs.msg import Image
from twizy_msgs.msg import BoundingBox, BoundingBoxes

def main():
    rospy.init_node('yolo')

    bbs = BoundingBoxes()

    def cb_image(img):
        bbs.image_header = img.header
        bbs.bounding_boxes = []

        # if len(bbs.bounding_boxes) > 0:
        pub_bbs.publish(bbs)

    pub_bbs = rospy.Publisher('bounding_boxes', BoundingBoxes, queue_size=1)

    rospy.Subscriber('image', Image, callback=cb_image, queue_size=1)

    rospy.spin()