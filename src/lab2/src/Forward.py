#!/usr/bin/env python


import cv2
from cv_bridge import CvBridge, CvBridgeError
import math
import matplotlib.pyplot as plt
import numpy as np
from pprint import pprint
import sys


from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped
import tf
from tf import transformations
import rospy

from util import _mask_img, _getDefaultBlobParams, DEFAULT_BLOB_PARAMS



blue_boundaries = [
  ((05, 100, 100), (35, 240, 220))
]

# TODO: Get the values for RED
red_boundaries = [
  ((05, 100, 100), (35, 240, 220))
]

SLEEP_TIME = 5.0


CAMERA_ANGLE = 0.0
CAMERA_FRAME_PARENT = 'camera_rgb_optical_frame'
CAMERA_FRAME_CHILD = 'base_link'


### For template creation
MAX_ANGLE = 0.34
NUM_TEMPLATES = 20
NUM_PTS = 150
V = 0.46  # Car's current velocity
CAR_LEN = 0.33


def create_template(steering):
  """ Uses the Kinematic Model to create a template using 'steering'
    as a constant steering angle."""

  init_pt = (0, 0)

  X = []
  Y = []

  last_pos = (0, 0)
  theta = 0
  for i in range(NUM_PTS):
    dt = 0.02
    delta_theta = V / CAR_LEN * np.sin(steering) * dt

    # Car should go straight if the steering angle is 0.
    if steering == 0.0:
      delta_x = V * dt
      delta_y = 0
    else:
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

  def __init__(self, control_pub, image_pub, params=_getDefaultBlobParams()):
    self.cvBridge = CvBridge()
    self.tl = tf.TransformListener()
    self.tb = tf.TransformBroadcaster()
    rospy.sleep(rospy.Duration(SLEEP_TIME))
    self.tl.waitForTransform(CAMERA_FRAME_PARENT, CAMERA_FRAME_CHILD, rospy.Time(), rospy.Duration(5.0))
    translation, rotation = self.tl.lookupTransform(CAMERA_FRAME_PARENT, CAMERA_FRAME_CHILD, rospy.Time())
    x, y, z = transformations.euler_from_quaternion(rotation)
    rot_matrix = transformations.euler_matrix(x + CAMERA_ANGLE, y, z)  # I am not positive about this


    # Calculate rotation matrix and translation vector
    translation = list(translation) + [1]
    rot_matrix[:, -1] = translation
    rot_matrix = np.array(rot_matrix)
    print(rot_matrix)

    self.control_pub = control_pub
    self.image_pub = image_pub

    templates = []
    self.robot_frames = []
    self.camera_frames = []
    self.pixel_frames = None
    self.point_frames = None

    # Flag to prevent callback from accessing non-existent frames
    self.preprocessed = False

    # Last Control for continual publishing when the robot gets lost.
    self.control = None

    # Discretize angles, one per template.
    self.discretized_angles = np.linspace(-MAX_ANGLE, MAX_ANGLE, NUM_TEMPLATES)
    for theta in self.discretized_angles:

      print theta
      Xw, Yw = create_template(theta)
      Zw = np.zeros(NUM_PTS)  # W.r.t car's frame (base link)
      ones = np.ones(NUM_PTS)  # Addition of 1 allows rotation multiplication

      # Convert from robot_frame to camera frame
      robot_frame = np.array([Xw, Yw, Zw, ones])
      camera_frame = rot_matrix.dot(robot_frame)
      self.robot_frames.append(robot_frame)
      self.camera_frames.append(camera_frame)

    # Plot Robot Frames
   #  Xs, Ys = np.array([]), np.array([])
   #  for rf in self.robot_frames:
   #    Xs = np.append(Xs, rf[0])
   #    Ys = np.append(Ys, rf[1])
   #
   #  plt.scatter(Xs, Ys)
   #  plt.show()
   #
   # # Plot Camera Frames
   #  Xc, Yc = np.array([]), np.array([])
   #  for cf in self.camera_frames:
   #    print cf
   #    Xc = np.append(Xc, cf[0])
   #    Yc = np.append(Yc, cf[2])
   #
   #
   #  plt.scatter(Xc, Yc)
   #  axes = plt.gca()
   #  axes.set_ylim([0, 2.0])
   #
   #  plt.show()


    # Blob parameters
    if cv2.__version__.startswith("3."):
      self.blobDetector = cv2.SimpleBlobDetector_create(params)
    else:
      self.blobDetector = cv2.SimpleBlobDetector(params)


  def image_cb(self, msg):
    if self.preprocessed:

      predicted_control = self.control #None at the start

      brg_img = self.cvBridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
      hsv_img = cv2.cvtColor(brg_img, cv2.COLOR_BGR2HSV)

      mask_img = _mask_img(hsv_img, red_boundaries)

      # Compute blobs.
      keypoints = self.blobDetector.detect(mask_img)
      max_keypoint = None
      for i, keypoint in enumerate(keypoints):
        if max_keypoint is None or keypoint.size > max_keypoint.size:
          max_keypoint = keypoint

      if max_keypoint is not None:
        x, y = max_keypoint.pt[0], max_keypoint.pt[1]
        self.visualize(mask_img, x, y)

        distances = np.abs(self.point_frames - x)
        min_dist_idx = np.argmin(distances)

        # Templates align with the angles that created them.
        predicted_control = self.discretized_angles[min_dist_idx]
        self.control = predicted_control
        print "CONTROL: ", predicted_control, "INDEX", min_dist_idx

      if predicted_control is not None:
        self.publish_controls(predicted_control)


  def publish_controls(self, steering_angle):
    ads = AckermannDriveStamped()
    ads.drive.steering_angle = steering_angle
    ads.drive.speed = V
    self.control_pub.publish(ads)


  def visualize(self, image, x, y):
    # Visualize an image to the object's given image_pub topic.
    if image is not None:

      if y is not None and x is not None:
        cv2.circle(image, (x, y), 15, (0, 255, 0), 3, cv2.LINE_AA)
      rosImg = self.cvBridge.cv2_to_imgmsg(image)
      self.image_pub.publish(rosImg)


  def k_cb(self, msg):
    if not self.preprocessed:

      k = msg.K
      self.pixel_frames = []
      self.point_frames = []

      K = np.array(list(k)).reshape(3, 3)

      U, V = [], []
      # Instantiate pixel frames from camera frames
      for i, camera_frame in enumerate(self.camera_frames):
        x_prime = camera_frame[0]
        y_prime = camera_frame[1]
        z_prime = camera_frame[2]

        # Instill camera intrinsics and convert
        # to pixel frame
        u = x_prime / z_prime
        v = y_prime / z_prime
        ones = np.ones(NUM_PTS)
        pixel_frame = np.array([u, v, ones])
        pixel_frame = K.dot(pixel_frame)

        self.pixel_frames.append(pixel_frame)

      # Take the last points here
      Us = np.array([])
      Vs = np.array([])
      for pixel_frame in self.pixel_frames:
        u = pixel_frame[0]
        v = pixel_frame[1]
        Us = np.append(Us, u)
        Vs = np.append(Vs, v)

        # Collect pointwise templates
        u_f = u[-1]
        v_f = v[-1]
        self.point_frames.append(u_f)

      # plt.scatter(Us, Vs)
      # axes = plt.gca()
      # axes.set_ylim([0, 480])
      # axes.set_xlim([0, 640])
      # axes.invert_yaxis()

      plt.show()

      self.point_frames = np.array(self.point_frames)
      self.preprocessed = True


