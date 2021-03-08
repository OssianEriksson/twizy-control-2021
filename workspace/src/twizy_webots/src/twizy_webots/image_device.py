import rospy

from sensor_msgs.msg import CameraInfo, Image

from math import pi, tan

import twizy_webots.util as util


def main(image_topic):
    """
    Images will be read from the "/<model_name>/<device>/<image_topic>" topic
    """

    # Initialize ROS node. There might be multiple versions of this node
    # running at once with the same name, anonymous=True allows this
    rospy.init_node('camera', anonymous=True)

    # Read parameters from ROS parameter server
    device = rospy.get_param('~device')
    frame_id = rospy.get_param('~frame_id', None)
    fps = rospy.get_param('fps', 30)
    fov = rospy.get_param('fov', pi / 2.0)
    distortion_model = rospy.get_param('distortion_model', 'plumb_bob')
    D = rospy.get_param('D', [0, 0, 0, 0, 0])
    R = rospy.get_param('R', [1, 0, 0,
                              0, 1, 0,
                              0, 0, 1])

    # Get webots model name
    model_name = util.model_name()

    # Attempt to enable the device
    if not util.enable(model_name, device, fps):
        rospy.logfatal('Unable to enable device {}'.format(device))
        return

    # Prepare CameraInfo message
    camerainfo = CameraInfo()
    camerainfo.distortion_model = distortion_model
    camerainfo.D = D
    camerainfo.R = R

    # Initialize publishers
    pub_image_raw = rospy.Publisher('/image_raw', Image, queue_size=1)
    pub_camera_info = rospy.Publisher('/camera_info', CameraInfo, queue_size=1)

    def cb_image(img):
        """
        Callback for hooking on images newly published by webots ROS controller.
        Once a new message is published and this function is called, we send
        out a corresponing camera_info message and redirect the image topic
        to a new topic
        """

        # Update frame_id
        img.header.frame_id = util.frame_id(img.header.frame_id, frame_id)

        # Copy image size and header. Syncing the header is what links the
        # camera info to a specific image (one camera info message should be
        # sent out per image)
        camerainfo.width = img.width
        camerainfo.height = img.height
        camerainfo.header = img.header

        # Calculate focal length
        f = 0.5 * img.width / tan(0.5 * fov)

        # Calculate optical center
        cx = img.width * 0.5
        cy = img.height * 0.5

        # Set matricies according to CameraInfo definition at
        # http://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html
        camerainfo.K = [f,  0, cx,
                        0,  f, cy,
                        0,  0,  1]
        camerainfo.P = [f,  0, cx, 0,
                        0,  f, cy, 0,
                        0,  0,  1, 0]

        # Publish camera info message
        pub_camera_info.publish(camerainfo)

        # Forward image to output topic
        pub_image_raw.publish(img)

    # Subscribe to PointStamped topic published by webots ROS controller
    image_name = '/{}/{}/{}'.format(model_name, device, image_topic)
    rospy.Subscriber(image_name, Image, callback=cb_image, queue_size=1)

    # Wait for shutdown
    rospy.spin()
