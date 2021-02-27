import rospy

import tf2_ros

from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped

br = tf2_ros.TransformBroadcaster()
t = TransformStamped()

t.header.frame_id = "map"
t.child_frame_id = "base_link"
t.transform.rotation.x = 0.0
t.transform.rotation.y = 0.0
t.transform.rotation.z = 0.0
t.transform.rotation.w = 1.0

def cb_gnss(msg):
    t.header.stamp = rospy.Time.now()
    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = msg.z

    br.sendTransform(t)

def main():
    rospy.init_node('twizy_state')

    rospy.Subscriber('/twizy_pose', Point, callback=cb_gnss)

    rospy.spin()
