import rospy
import rospkg

import yaml
import os

from webots_ros.srv import set_int, set_float
from std_msgs.msg import String
from twizy_msgs.msg import CarControl


def enable_device(robot_name, device, ups):
    enable_name = '/{}/{}/enable'.format(robot_name, device)

    rospy.wait_for_service(enable_name)
    enable = rospy.ServiceProxy(enable_name, set_int)

    if not enable(int(round(1000.0 / ups))):
        rospy.logfatal('Unable to enable device {}'.format(device))


def get_position_control_service(robot_name, device):
    return rospy.ServiceProxy('/{}/{}/set_position'.format(robot_name, device),
                              set_float)


def get_velocity_control_service(robot_name, device):
    set_position = get_position_control_service(robot_name, device)
    set_position.wait_for_service()
    set_position(float('inf'))
    set_position.close()

    return rospy.ServiceProxy('/{}/{}/set_velocity'.format(robot_name, device),
                              set_float)


def main():
    rospy.init_node('controller')

    robot_name = rospy.wait_for_message('/model_name', String).data

    realsense_fps = rospy.get_param('realsense_fps', 1)
    gnss_ups = rospy.get_param('gnss_ups', 1)

    enable_device(robot_name, 'front_realsense_depth_camera', realsense_fps)
    enable_device(robot_name, 'front_realsense_aligned_depth_to_color_camera',
                  realsense_fps)

    enable_device(robot_name, 'left_gnss', gnss_ups)
    enable_device(robot_name, 'right_gnss', gnss_ups)

    left_steering = get_position_control_service(robot_name,
                                                 'front_left_hinge_motor')
    right_steering = get_position_control_service(robot_name,
                                                  'front_right_hinge_motor')
    rear_left_weel = get_velocity_control_service(robot_name,
                                                  'rear_left_wheel_motor')
    rear_right_weel = get_velocity_control_service(robot_name,
                                                  'rear_right_wheel_motor')

    # Get path to twizy_description package
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('twizy_description')

    # Get path to robot properties file
    yaml_filename = 'twizy_properties.yaml'
    yaml_file = os.path.join(package_path, 'config', yaml_filename)

    # Parese YAML file
    with open(yaml_file, 'r') as f:
        props = yaml.safe_load(f)

    wheel_radius = props['wheel']['radius']
    max_speed = props['wheel']['max_speed']
    max_angle = props['wheel']['max_steering_angle']

    def cb_control(msg):
        angle = max(min(msg.angle, max_angle), -max_angle)
        speed = max(min(msg.speed, max_speed), -max_speed)

        angular_vel = speed / wheel_radius

        if not left_steering(angle):
            rospy.logwarn('Unable to set left side steering angle')
        if not right_steering(angle):
            rospy.logwarn('Unable to set right side steering angle')
        if not rear_left_weel(angular_vel):
            rospy.logwarn('Unable to set rear left wheel speed')
        if not rear_right_weel(angular_vel):
            rospy.logwarn('Unable to set rear left wheel speed')

    # Subscribe to topic where reference control values will be published
    rospy.Subscriber('car_control', CarControl, cb_control, queue_size=1)

    # Wait for shutdown
    rospy.spin()
