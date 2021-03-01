import rospy

from webots_ros.srv import set_int
from std_msgs.msg import String


def enable_device(robot_name, device, ups):
    enable_name = '/{}/{}/enable'.format(robot_name, device)

    rospy.wait_for_service(enable_name)
    enable = rospy.ServiceProxy(enable_name, set_int)

    if not enable(int(round(1000.0 / ups))):
        rospy.logfatal('Unable to enable device {}'.format(device))


def main():
    rospy.init_node('controller')

    robot_name = rospy.wait_for_message('/model_name', String).data

    realsense_fps = rospy.get_param('realsense_fps', 30)
    gnss_ups = rospy.get_param('gnss_ups', 30)

    enable_device(robot_name, 'front_realsense_depth_camera', realsense_fps)
    enable_device(robot_name, 'front_realsense_aligned_depth_to_color_camera',
                  realsense_fps)

    enable_device(robot_name, 'left_gnss', gnss_ups)
    enable_device(robot_name, 'right_gnss', gnss_ups)

    rospy.spin()
