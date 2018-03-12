#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Quaternion, PolygonStamped,Polygon, Point32, PoseWithCovarianceStamped, PointStamped
import tf.transformations
import tf
import matplotlib.pyplot as plt
import csv


def angle_to_quaternion(angle):
    """Convert an angle in radians into a quaternion _message_."""
    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))

def quaternion_to_angle(q):
    """Convert a quaternion _message_ into an angle in radians.
    The angle represents the yaw.
    This is not just the z component of the quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    roll, pitch, yaw = tf.transformations.euler_from_quaternion((x, y, z, w))
    return yaw

def rotation_matrix(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.matrix([[c, -s], [s, c]])


def world_to_map(pose, map_info):
    # equivalent to map_to_grid(world_to_map(pose))
    # operates in place
    scale = map_info.resolution
    angle = -quaternion_to_angle(map_info.origin.orientation)
    config = [0.0, 0.0, 0.0]
    # translation
    config[0] = (1.0/float(scale))*(pose[0] - map_info.origin.position.x)
    config[1] = (1.0/float(scale))*(pose[1] - map_info.origin.position.y)
    config[2] = pose[2]
   

    # rotation
    c, s = np.cos(angle), np.sin(angle)
    # we need to store the x coordinates since they will be overwritten
    temp = np.copy(config[0])
    config[0] = int(c*config[0] - s*config[1])
    config[1] = int(s*temp       + c*config[1])
    config[2] += angle
    
    return config


def map_to_world(pose, map_info):
    scale = map_info.resolution
    angle = quaternion_to_angle(map_info.origin.orientation)
    config = [0.0, 0.0, 0.0]

    # rotation
    c, s = np.cos(angle), np.sin(angle)
    # we need to store the x coordinates since they will be overwritten
    temp = np.copy(pose[0])
    config[0] = (c*pose[0] - s*pose[1])
    config[1] = map_info.height - (s*temp       + c*pose[1])

    config[0] *= float(scale)
    config[1] *= float(scale)

    # translate
    config[0] += map_info.origin.position.x
    config[1] += map_info.origin.position.y
    config[2] = pose[2] + angle

    return config

# returns numpy array with a shape of (N,2)
def csv_to_configs(file_path):

    with open(file_path, 'r') as fin:
        csv_reader = csv.reader(fin, delimiter=',')
        way_points = [row for row in csv_reader][1:]  # Skip header

    return np.array(way_points).astype(np.int)

def angle_between_points(point1, point2):
    x = float(point2[0] - point1[0])
    y = float(point1[1] - point2[1])
    angle = np.arctan(y/x)
    if angle < 0.0:
        angle += np.pi * 2
    return angle



