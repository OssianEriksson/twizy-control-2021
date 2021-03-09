import rospy

import threading
import struct
import math

from astar import AStar
import numpy as np

import tf2_geometry_msgs
import tf2_ros
from twizy_msgs.msg import MapWithClasses
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class MazeSolver(AStar):
    def __init__(self, maze):
        self.maze = maze
        self.width = maze.shape[1]
        self.height = maze.shape[0]

    def heuristic_cost_estimate(self, n1, n2):
        """computes the 'direct' distance between two (x,y) tuples"""
        (x1, y1) = n1
        (x2, y2) = n2
        return math.hypot(x2 - x1, y2 - y1)

    def distance_between(self, n1, n2):
        """this method always returns 1, as two 'neighbors' are always adajcent"""
        return 1

    def neighbors(self, node):
        """ for a given coordinate in the maze, returns up to 4 adjacent(north,east,south,west)
            nodes that can be reached (=any adjacent coordinate that is not a wall)
        """
        x, y = node
        return [(nx, ny) for nx, ny in [(x, y - 1), (x, y + 1), (x - 1, y), (x + 1, y), (x + 1, y + 1), (x + 1, y - 1), (x - 1, y + 1), (x - 1, y - 1)] if nx < 0 or nx >= self.width or ny < 0 or ny >= self.height or self.maze[ny][nx] < 10]


class PathPlannerInput:
    def __init__(self, tfBuffer, base_link):
        self.tfBuffer = tfBuffer
        self.base_link = base_link
        self.map = None
        self.goal = None

    def ready(self):
        return (self.map is not None) and (self.goal is not None)


def _get_path(pp):
    position = pp.tfBuffer.lookup_transform(
        pp.map.header.frame_id,
        pp.base_link,
        pp.map.header.stamp,
        rospy.Duration(1.0)
    ).transform.translation

    goal_transform = pp.tfBuffer.lookup_transform(
        pp.map.header.frame_id,
        pp.goal.header.frame_id,
        pp.map.header.stamp,
        rospy.Duration(1.0)
    )

    goal = tf2_geometry_msgs.do_transform_pose(pp.goal, goal_transform)

    start_float = ((position.x - pp.map.x) / pp.map.resolution,
                   (position.y - pp.map.y) / pp.map.resolution)

    start = (int(start_float[0]), int(start_float[1]))

    goal = (int((goal.pose.position.x - pp.map.x) / pp.map.resolution),
            int((goal.pose.position.y - pp.map.y) / pp.map.resolution))

    occupance = pp.map.occupance_probability
    occupance = np.array(struct.unpack('{}B'.format(
        len(occupance)), occupance)).reshape((pp.map.height, pp.map.width))

    foundPath = list(MazeSolver(occupance).astar(start, goal))

    path = Path()
    path.header.frame_id = pp.map.header.frame_id
    path.header.stamp = rospy.Time.now()

    poses = []

    x0, y0 = start_float

    for x, y in foundPath:
        pose = PoseStamped()
        pose.header = path.header

        pose.pose.position.x = x * pp.map.resolution + pp.map.x
        pose.pose.position.y = y * pp.map.resolution + pp.map.y
        pose.pose.position.z = 0

        angle = math.atan2(y - y0, x - x0) / 2

        x0, y0 = x, y

        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = math.sin(angle)
        pose.pose.orientation.w = math.cos(angle)

        poses.append(pose)

    path.poses = poses

    return path


def main():
    rospy.init_node('class_path_planner')

    base_link = rospy.get_param('~base_link', 'base_link')

    # Initialize tf buffer for buffering transforms between frames
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    lock = threading.Lock()

    pp_input = PathPlannerInput(tfBuffer, base_link)

    pub_path = rospy.Publisher('path', Path, queue_size=1)

    def publish_path():
        if pp_input.ready():
            pub_path.publish(_get_path(pp_input))

    def cb_map(map_msg):
        with lock:
            pp_input.map = map_msg
            publish_path()

    def cb_goal(goal_msg):
        with lock:
            pp_input.goal = goal_msg
            publish_path()

    rospy.Subscriber('goal', PoseStamped, callback=cb_goal, queue_size=1)
    rospy.Subscriber('map_with_classes', MapWithClasses,
                     callback=cb_map, queue_size=1)

    rospy.spin()
