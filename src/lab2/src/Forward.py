#!/usr/bin/env python


import matplotlib.pyplot as plt
import numpy as np
from pprint import pprint

from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
from util import _mask_img
import tf
from tf import transformations
from geometry_msgs.msg import TransformStamped
import rospy
import math

# TODO: Change to RED
boundaries = [
  ((05, 100, 100), (35, 240, 220))
]

SLEEP_TIME = 3.0

NUM_TEMPLATES = 20
MAX_ANGLE = 0.34

CAMERA_ANGLE = 0.0

CAMERA_FRAME = 'camera_rgb_optical_frame'
# CAMERA_FRAME = 'camera_link'




from util import _mask_img


MAX_ANGLE = 0.34
NUM_TEMPLATES = 25
NUM_PTS = 200
V = 0.5  # Car's current velocity
CAR_LEN = 0.33


def create_template(steering):
    init_pt = (0, 0)

    X = []
    Y = []

    last_pos = (0, 0)
    theta = 0
    for i in range(NUM_PTS):
        dt = 0.02
        delta_theta = V / CAR_LEN * np.sin(steering) * dt
        delta_x = CAR_LEN / np.sin(steering) * (np.sin(theta + delta_theta) - np.sin(theta))
        delta_y = CAR_LEN / np.sin(steering) * (np.cos(theta) - np.cos(theta + delta_theta))

        x = delta_x + last_pos[0]
        y = delta_y + last_pos[1]

        X.append(x)
        Y.append(y)

        theta += delta_theta
        last_pos = (x, y)


    return X, Y



class ForwardController(object):

  def __init__(self, control_pub):
    self.cvBridge = CvBridge()
    self.tl = tf.TransformListener()
    self.tb = tf.TransformBroadcaster()
    rospy.sleep(rospy.Duration(SLEEP_TIME))
    translation, rotation = self.tl.lookupTransform(CAMERA_FRAME, 'base_link', rospy.Time())
    x, y, z = transformations.euler_from_quaternion(rotation)
    rot_matrix = transformations.euler_matrix(x + CAMERA_ANGLE, y, z)  # I am not positive about this

    translation = list(translation) + [1]
    rot_matrix[:, -1] = translation
    rot_matrix = np.array(rot_matrix)
    print(rot_matrix)

    # tranform = TransformStamped()
    # tranform.transform.rotation.x = rotation[0]
    # tranform.transform.rotation.y = rotation[1]
    # tranform.transform.rotation.z = rotation[2]
    # tranform.transform.rotation.w = rotation[3]
    # tranform.transform.translation.x = translation[0]
    # tranform.transform.translation.y = translation[1]
    # tranform.transform.translation.z = translation[2]
    # tranform.header.frame_id = 'base_link'
    # tranform.child_frame_id = 'my_frame'
    # new_rotation = transformations.euler_from_quaternion(rotation)
    # new_rotation[0] += math.pi
    # new_rotation = tuple(new_rotation)
    # x, y, z = new_rotation
    # x += 0.1
    # translation[0] += 0
    # new_rotation = transformations.quaternion_from_euler(x, y, z)
    # pprint((x, y, z))
    # pprint(rotation)

    #self.tl.setTransform(tranform)
    # self.tb.sendTransform(translation, rotation, rospy.Time(), CAMERA_FRAME, 'camera_link')
    self.control_pub = control_pub


    templates = []

    # Discretize angles, one per template.
    self.discretized_angles = np.linspace(-MAX_ANGLE, MAX_ANGLE, NUM_TEMPLATES)
    Xs, Ys = [], []
    for theta in self.discretized_angles:

        print theta
        X, Y = create_template(theta)
        Xs += X
        Ys += Y

    template_img = plt.scatter(Xs, Ys)
    plt.show()


  def image_cb(self, msg):


    brg_img = self.cvBridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    hsv_img = cv2.cvtColor(brg_img, cv2.COLOR_BGR2HSV)
    mask_img = _mask_img(hsv_img, boundaries)
    #pprint(msg)


  def visulize(self):
      pass





test = ForwardController("hello")



