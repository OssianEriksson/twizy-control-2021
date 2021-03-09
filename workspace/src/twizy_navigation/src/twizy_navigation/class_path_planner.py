import rospy

import threading
import numpy as np

from astar import AStar

import tf2_geometry_msgs
from twizy_msgs.msg import MapWithClasses
from geometry_msgs.msg import PoseStamped
from nav_msgs import Path

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
        return [(nx, ny) for nx, ny in [(x, y - 1), (x, y + 1), (x - 1, y), (x + 1, y)] if 0 <= nx < self.width and 0 <= ny < self.height and self.maze[ny][nx] < 10]


class PathPlannerInput:
    def __init__(self, tfBuffer, base_link):
        self.tfBuffer = tfBuffer
        self.base_link = base_link
        self.map = None
        self.goal = None
    
    def ready():
        return self.map is not None and self.goal is not None


def _get_path(pp):
    position = tfBuffer.lookup_transform(
        pp.map.header.frame_id,
        pp.base_link,
        pp.map.header.stamp,
        rospy.Duration(1.0)
    ).transform.translation

    goal = tfBuffer.lookup_transform(
        pp.map.header.frame_id,
        pp.goal.header.frame_id,
        pp.map.header.stamp,
        rospy.Duration(1.0)
    ).transform.translation

    start = ((position.x - pp.map.x) / pp.map.resolution,
    (position.x - pp.map.x) / pp.map.resolution)


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

    rospy.Subscriber('goal', PoseStamped, callback = cb_goal, queue_size = 1)
    rospy.Subscriber('map_with_classes', MapWithClasses,
                     callback = cb_map, queue_size = 1)
