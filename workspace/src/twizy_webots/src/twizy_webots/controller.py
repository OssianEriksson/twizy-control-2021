import rospy

from webots_ros.srv import set_int
from std_msgs.msg import String


def enable_device(robot_name, device, update_rate):
    enable_name = '/{}/{}/enable'.format(robot_name, device)

    rospy.wait_for_service(enable_name)
    enable = rospy.ServiceProxy(enable_name, set_int)

    if not enable(update_rate):
        rospy.logfatal('Unable to enable device {}'.format(device))


def main():
    rospy.init_node('controller')

    robot_name = rospy.wait_for_message('/model_name', String).data

    camera_fps = 30
    camera_delay = 1000 / camera_fps # (ms)

    enable_device(robot_name, 'front_realsense_depth_camera', camera_delay)
    enable_device(robot_name, 'front_realsense_aligned_depth_to_color_camera',
                  camera_delay)

    rospy.spin()
