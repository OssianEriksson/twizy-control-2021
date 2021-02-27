import rospy

from geometry_msgs.msg import Point, PoseWithCovarianceStamped


def main():
    rospy.init_node('twizy_pose')

    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = 'odom'
    pose.pose.pose.orientation.x = 0.0
    pose.pose.pose.orientation.y = 0.0
    pose.pose.pose.orientation.z = 0.0
    pose.pose.pose.orientation.w = 1.0

    pub = rospy.Publisher('gnss/pose', PoseWithCovarianceStamped, queue_size=10)

    def cb(msg):
        pose.header.stamp = rospy.Time.now()
        pose.pose.pose.position.x = -msg.x
        pose.pose.pose.position.y = msg.y
        pose.pose.pose.position.z = msg.z

        pub.publish(pose)

    rospy.Subscriber('/twizy_pose', Point, callback=cb)

    rospy.spin()
