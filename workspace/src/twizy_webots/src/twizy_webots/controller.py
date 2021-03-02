import rospy
import rospkg

import yaml
import os

from webots_ros.srv import set_int, set_float
from std_msgs.msg import String
from twizy_msgs.msg import CarControl


def enable_device(robot_name, device, ups):
    """
    Calls enable on a webots device with name device belonging to the robot
    with name robot_name. The device is told to update at ups updates per
    second
    """

    enable_name = '/{}/{}/enable'.format(robot_name, device) # Name of service

    # Initialize service and wait for it to become available
    enable = rospy.ServiceProxy(enable_name, set_int)
    enable.wait_for_service()

    # Attempt to enable the device
    if not enable(int(round(1000.0 / ups))):
        rospy.logfatal('Unable to enable device {}'.format(device))


def get_position_control_service(robot_name, device):
    """
    Returns a set_position ServiceProxy for interfacing with the webots device
    with name device of robot with name robot_name
    """

    return rospy.ServiceProxy('/{}/{}/set_position'.format(robot_name, device),
                              set_float)


def get_velocity_control_service(robot_name, device):
    """
    Disables position control and returns a set_velocity ServiceProxy for
    interfacing with the webots device with name device of robot with name
    robot_name
    """

    # Disable position control
    set_position = get_position_control_service(robot_name, device)
    set_position.wait_for_service()
    set_position(float('inf'))  # This is what actually does the disabling
    set_position.close()  # Close isn't really needed here,
                          # included for good measure

    return rospy.ServiceProxy('/{}/{}/set_velocity'.format(robot_name, device),
                              set_float)


def main():
    # Initialize ROS node
    rospy.init_node('controller')

    # Get the robot's name from topic advertised by webots
    robot_name = rospy.wait_for_message('/model_name', String).data

    realsense_fps = rospy.get_param('~realsense_fps', 30.0)
    gnss_ups = rospy.get_param('~gnss_ups', 30.0)

    # Tell webots ROS controller to publish image topics
    enable_device(robot_name, 'front_realsense_depth_camera', realsense_fps)
    enable_device(robot_name, 'front_realsense_aligned_depth_to_color_camera',
                  realsense_fps)

    # Tell webots ROS controller to publish GNSS topics
    enable_device(robot_name, 'left_gnss', gnss_ups)
    enable_device(robot_name, 'right_gnss', gnss_ups)

    # Initalize services
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
        """
        Sets simulation model's steering angle and velocity
        """

        # Clamp value within allowed bounds
        angle = max(min(msg.angle, max_angle), -max_angle)
        speed = max(min(msg.speed, max_speed), -max_speed)

        # Convert from linear to angular velocity
        angular_vel = speed / wheel_radius

        # Set steering angles
        # TODO: Ackermann
        if not left_steering(angle):
            rospy.logwarn('Unable to set left side steering angle')
        if not right_steering(angle):
            rospy.logwarn('Unable to set right side steering angle')

        # Set rear wheel speeds
        if not rear_left_weel(angular_vel):
            rospy.logwarn('Unable to set rear left wheel speed')
        if not rear_right_weel(angular_vel):
            rospy.logwarn('Unable to set rear left wheel speed')

    # Subscribe to topic where reference control values will be published
    # Increasing the queue_size will create INSANE amount of accumulative input
    # lag for some reason (took way too long to figure that out...)
    rospy.Subscriber('car_control', CarControl, cb_control, queue_size=1)

    # Wait for shutdown
    rospy.spin()
