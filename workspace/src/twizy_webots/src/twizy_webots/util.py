import rospy

from webots_ros.srv import set_int
from std_msgs.msg import String


def enable(model_name, device, ups):
    """
    Calls enable on a webots device with name device belonging to the robot
    with name model_name. The device is told to update at ups updates per
    second
    """

    enable_name = '/{}/{}/enable'.format(model_name, device)  # Name of service
    enable = rospy.ServiceProxy(enable_name, set_int)

    # Wait for service to become available
    enable.wait_for_service()

    return enable(int(round(1000.0 / ups)))


def frame_id(frame_id, new_frame_id=None):
    """
    If new_frame_id is not None, return new_frame_id. Otherwise remove the
    prefix which the default webots ROS controller adds to frame_id and return
    the result
    """

    return frame_id.split('/')[-1] if new_frame_id is None else new_frame_id

def model_name(timeout=None):
    """
    Returns name of webots model, either from parameter server (/model_name) or
    from /model_name topic.

    timeout time in seconds or ROS Duration
    """

    return rospy.get_param(
        '/model_name',
        rospy.wait_for_message('/model_name', String, timeout).data
    )
