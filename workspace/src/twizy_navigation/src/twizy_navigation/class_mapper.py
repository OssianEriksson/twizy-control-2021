import rospy

import threading
import numpy as np
from numpy import floor, arctan, sqrt, cos, pi, ceil

from tf.transformations import quaternion_matrix, translation_matrix
from cv_bridge import CvBridge
import message_filters
import tf2_ros

from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image, CameraInfo
from twizy_msgs.msg import BoundingBoxes, MapWithClasses


def shift(A, shift, axis, paddning=None):
    if shift == 0:
        return

    A[:] = np.roll(A, shift, axis=axis)

    axis_indexer = [slice(None)] * A.ndim
    if shift > 0:
        axis_indexer[axis] = slice(None, shift)
    else:
        axis_indexer[axis] = slice(shift, None)

    A[tuple(axis_indexer)] = paddning


def main():
    rospy.init_node('class_mapper')

    inputs = rospy.get_param('~inputs')
    grid_resolution = rospy.get_param('~grid_resolution', 1.0)
    grid_radius = rospy.get_param('~grid_radius', 20)
    frequency = rospy.get_param('~frequency', 1.0)
    base_link = rospy.get_param('~base_link', 'base_link')
    fixed_frame = rospy.get_param('~fixed_frame', 'map')
    floor_level = rospy.get_param('~floor_level', 0.5)
    grid_initial = rospy.get_param('~grid_initial', 0.0)
    max_range = rospy.get_param(
        '/twizy_properties/realsense_d435/depth/max_range')
    classes = rospy.get_param(
        '~classes', [{'name': 'static', 'classes': ['chair']}])

    class_names = []

    for i, clazz in enumerate(classes):
        class_names.append(clazz['name'])

    l = len(class_names) + 1

    grid_dim = 2 * grid_radius + 1
    grid = np.zeros((l, grid_dim, grid_dim))
    grid[:] = grid_initial

    xy = [0.0, 0.0]

    # Initialize tf buffer for buffering transforms between frames
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    map_msg = MapWithClasses()
    map_msg.header.frame_id = fixed_frame
    map_msg.resolution = grid_resolution
    map_msg.width = grid_dim
    map_msg.height = grid_dim
    map_msg.class_names = class_names

    bridge = CvBridge()

    lock = threading.Lock()

    def callback(bbs, img, info):
        position = tfBuffer.lookup_transform(
            fixed_frame,
            base_link,
            img.header.stamp,
            rospy.Duration(1.0)
        ).transform.translation

        transform = tfBuffer.lookup_transform(
            fixed_frame,
            img.header.frame_id,
            img.header.stamp,
            rospy.Duration(1.0)
        ).transform

        with lock:
            dx_grid = int(floor((position.x - xy[0]) /
                                grid_resolution - grid_radius))
            dy_grid = int(floor((position.y - xy[1]) /
                                grid_resolution - grid_radius))
            xy[0] += dx_grid * grid_resolution
            xy[1] += dy_grid * grid_resolution

            shift(grid, -dx_grid, 2, grid_initial)
            shift(grid, -dy_grid, 1, grid_initial)

            depth = bridge.imgmsg_to_cv2(img)

            cx = info.P[2] + info.P[3]
            cy = info.P[6] + info.P[7]
            fx_inv = 1.0 / info.P[0]
            fy_inv = 1.0 / info.P[5]

            q = transform.rotation
            r = transform.translation
            R = quaternion_matrix([transform.rotation.x,
                                   transform.rotation.y,
                                   transform.rotation.z,
                                   transform.rotation.w])
            T = translation_matrix([transform.translation.x - xy[0],
                                    transform.translation.y - xy[1],
                                    transform.translation.z])
            S = np.eye(4)
            S[:6:5, :6:5] = 1.0 / grid_resolution

            STR = np.matmul(S, np.matmul(T, R))[:3]

            p1 = STR[:, 3]
            p_2d, p1_2d = np.zeros(2), p1[:2]

            for i in range(0, len(depth) * len(depth[0])):
                px = i % img.width
                py = int(i / img.width)

                z = depth[py][px]

                if z >= max_range:
                    continue

                x = z * fx_inv * (px + 0.5 - cx)
                y = z * fy_inv * (py + 0.5 - cy)

                p0 = np.matmul(STR, [x, y, z, 1])
                d_floor = position.z + floor_level - p0[2]
                if d_floor > 0:
                    delta = p1 - p0
                    p0 += (p1 - p0) * d_floor / delta[2]
                else:
                    rospy.loginfo(d_floor)
                    grid[0, int(floor(p0[1])), int(floor(p0[0]))] += 100

                p_2d[:] = p0[:2]
                delta = p1_2d - p_2d
                step = max(abs(delta))
                delta /= step

                for j in range(0, int(step)):
                    p_2d += delta
                    grid[:, int(floor(p_2d[1])),
                         int(floor(p_2d[0]))] *= 0.9

            np.minimum(grid, 255.999, out=grid)

    for inpt in inputs:
        sub_bbs = message_filters.Subscriber(
            inpt['bounding_boxes'], BoundingBoxes)
        sub_img = message_filters.Subscriber(inpt['depth_image'], Image)
        sub_info = message_filters.Subscriber(inpt['camera_info'], CameraInfo)

        ts = message_filters.TimeSynchronizer([sub_bbs, sub_img, sub_info], 10)
        ts.registerCallback(callback)

    pub_map = rospy.Publisher('map_with_classes', MapWithClasses, queue_size=1)

    rate = rospy.Rate(frequency)

    while not rospy.is_shutdown():
        with lock:
            map_msg.header.stamp = rospy.Time.now()
            map_msg.x = xy[0]
            map_msg.y = xy[1]
            map_msg.classes = np.argmax(grid[1:]).flatten().tolist()
            map_msg.class_probability = np.max(
                grid[1:]).flatten().astype(np.uint8).tolist()
            map_msg.occupance_probability = grid[0].flatten().astype(np.uint8).tolist()

            pub_map.publish(map_msg)

        rate.sleep()
