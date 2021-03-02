import rospy

from sensor_msgs.msg import CameraInfo, Image

from math import tan


def main():
    # Initialize ROS node. There might be multiple versions of this node
    # running at once with the same name, anonymous=True allows this
    rospy.init_node('publish_camera_info', anonymous=True)

    # Read parameters from ROS parameter server
    fov = rospy.get_param('~fov')
    image_topic = rospy.get_param('~image_topic')

    msg = CameraInfo()
    msg.distortion_model = 'plumb_bob'  # Basic camera distortion model
    msg.D = [0, 0, 0, 0, 0]  # Parameters for plumb_bob. Don't know what these
                             # do, but setting them all to 0 seems to work
    msg.R = [1, 0, 0, # Identity rotation matrix
             0, 1, 0,
             0, 0, 1]

    # Topic to publish camera info to. If image_ropic is e.g.
    # /camera/test/image_raw then we will publish camera info to
    # /camera/test/camera_info
    info_topic = '/'.join(image_topic.split('/')[:-1]) + '/camera_info'

    # Initialize publisher for camera info messages
    pub = rospy.Publisher(info_topic, CameraInfo, queue_size=1, latch=True)

    def cb(img):
        """
        Callback for hooking on newly published images on ~image_topic. Once
        a new message is published and this function is called, we send out
        a corresponing camera_info message
        """

        # Copy image size and header. Syncing the header is what links the
        # camera info to a specific image (one camera info message should be
        # sent out per image)
        msg.width = img.width
        msg.height = img.height
        msg.header = img.header

        # Calculate focal length
        f = 0.5 * img.width / tan(0.5 * fov)

        # Calculate optical center
        cx = img.width * 0.5
        cy = img.height * 0.5

        # Set matricies according to CameraInfo definition at
        # http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/
        # CameraInfo.html
        msg.K = [f,  0, cx,
                 0,  f, cy,
                 0,  0,  1]
        msg.P = [f,  0, cx, 0,
                 0,  f, cy, 0,
                 0,  0,  1, 0]

        # Publish camera info message
        pub.publish(msg)

    # Hook on new images beeing published to ~image_topic
    rospy.Subscriber(image_topic, Image, queue_size=1, callback=cb)

    # Wait for shutdown
    rospy.spin()
