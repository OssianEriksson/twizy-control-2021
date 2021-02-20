import math

import rospy

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from rosgraph_msgs.msg import Clock

from controller import Robot


class PointCloud2Camera:
    def __init__(self, depth, color):
        if depth.getFov() != color.getFov():
            raise RuntimeError("Depth and color FOV:s must match!")

        if (depth.getWidth() != color.getWidth() or
                depth.getHeight() != color.getHeight()):
            raise RuntimeError("Depth and color resolutions must match!")

        self.depth = depth
        self.color = color
        self.fov = depth.getFov()
        self.width = depth.getWidth()
        self.height = depth.getHeight()
        self.max_range = depth.getMaxRange()

    def create_point_cloud2(self, frame_id):
        t = math.tan(self.fov / 2)

        a = 2.0 * t / self.width
        bx = -(self.width - 1) / 2.0
        by = -(self.height - 1) / 2.0

        depth = self.depth.getRangeImageArray()
        color = self.color.getImageArray()

        points = []

        for xi in range(0, self.width):
            for yi in range(0, self.height):
                z = depth[xi][yi]

                if z == self.max_range:
                    continue

                x = z * a * (xi + bx)
                y = -z * a * (yi + by)

                rgb = color[xi][yi]
                bgr = (rgb[2] << 16) + (rgb[1] << 8) + rgb[0]

                points.append([x, z, y, bgr])

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('rgb', 12, PointField.UINT32, 1)]

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id
        return point_cloud2.create_cloud(header, fields, points)


class PeriodicAction:
    def __init__(self, rate):
        self.t = rospy.Time.now()
        self.dt = rospy.Duration(1.0 / rate)
    
    def update(self):
        _t = rospy.Time.now()
        if _t >= self.t:
            self.t = max(self.t + self.dt, _t)
            return True
        return False



def main():
    rospy.init_node('twizy_webots', anonymous=True)

    controls = {
        'speed': 0,
        'steer_left': 0,
        'steer_right': 0,
    }

    def cb_control(msg):
        wheel_radius = 0.25
        wheelbase = 1.4
        track = 1

        controls['speed'] = msg.data[0] / wheel_radius

        angle = min(max(-msg.data[1], -89.99), 89.99) * math.pi / 180

        centerline_to_intersection = wheelbase / \
            math.tan(angle) if abs(angle) > 1e-10 else float('inf')

        controls['steer_left'] = math.atan(
            wheelbase / (centerline_to_intersection - track / 2))
        controls['steer_right'] = math.atan(
            wheelbase / (centerline_to_intersection + track / 2))

        

    rospy.loginfo('REMOVE OUTPUT="SCREEN"')

    clock = rospy.Publisher('/clock', Clock, queue_size=1)
    pub_points = rospy.Publisher(
        '/camera/depth/color/points', PointCloud2, queue_size=1)
    pub_point = rospy.Publisher('twizy_pose', Point, queue_size=1)
    
    rospy.Subscriber('/car_control', Float64MultiArray, callback=cb_control)

    robot = Robot()

    time_step = int(robot.getBasicTimeStep())

    left_steer = robot.getDevice('left_steer')
    right_steer = robot.getDevice('right_steer')

    left_drive = robot.getDevice('left_drive')
    right_drive = robot.getDevice('right_drive')
    left_drive.setPosition(float('inf'))   # Enable velocity control
    right_drive.setPosition(float('inf'))  # Enable velocity control
    left_drive.setVelocity(0)
    right_drive.setVelocity(0)

    center_gnss = robot.getDevice('center_gnss')
    center_gnss.enable(time_step)

    front_depth = robot.getDevice('front_depth')
    front_aligned_color = robot.getDevice('front_aligned_depth_to_color')
    front_depth.enable(time_step)
    front_aligned_color.enable(time_step)

    front_realsense = PointCloud2Camera(front_depth, front_aligned_color)

    publish_cameras = PeriodicAction(30)

    while robot.step(time_step) != -1 and not rospy.is_shutdown():
        clock.publish(Clock(rospy.Time(robot.getTime())))

        left_steer.setPosition(controls['steer_left'])
        right_steer.setPosition(controls['steer_right'])

        left_drive.setVelocity(controls['speed'])
        right_drive.setVelocity(controls['speed'])

        p = center_gnss.getValues()

        pub_point.publish(Point(p[0], p[2], p[1]))

        if publish_cameras.update():
            pc2 = front_realsense.create_point_cloud2("/front_realsense")
            pub_points.publish(pc2)
