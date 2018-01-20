#!/usr/bin/env python

import rospy
import numpy as np
import tf.transformations
from pprint import pprint

# THESE FUNCTIONS MAY OR MAY NOT BE HELPFUL IN YOUR IMPLEMENTATION
# IMPLEMENT/USE AS YOU PLEASE

def angle_to_quaternion(angle):
  quat = tf.transformations.quaternion_from_euler(0, 0, angle)
  return quat

def quaternion_to_angle(q):
  euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
  # pprint(euler)
  return euler[2]

def map_to_world(poses,map_info):
  pass
  
def world_to_map(poses, map_info):
  pass
