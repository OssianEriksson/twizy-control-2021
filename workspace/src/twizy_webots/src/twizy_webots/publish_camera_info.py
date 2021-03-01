import rospy

from sensor_msgs.msg import CameraInfo, Image

from math import tan

def main():
    rospy.init_node('publish_camera_info', anonymous=True)

    fov = rospy.get_param('~fov')

    msg = CameraInfo()
    msg.distortion_model = 'plumb_bob'
    msg.D = [0, 0, 0, 0, 0]
    msg.R = [1, 0, 0,
             0, 1, 0,
             0, 0, 1]

    image_topic = rospy.get_param('~image_topic')

    info_topic = '/'.join(image_topic.split('/')[:-1]) + '/camera_info'

    pub = rospy.Publisher(info_topic, CameraInfo, queue_size=1, latch=True)

    def cb(img):
        msg.width = img.width
        msg.height = img.height
        msg.header = img.header

        f = 0.5 * img.width / tan(0.5 * fov)
        cx = img.width * 0.5
        cy = img.height * 0.5

        msg.K = [f,  0, cx,
                 0,  f, cy,
                 0,  0,  1]
        msg.P = [f,  0, cx, 0,
                 0,  f, cy, 0,
                 0,  0,  1, 0]

        pub.publish(msg)

    rospy.Subscriber(image_topic, Image, queue_size=1, callback=cb)

    rospy.spin()
