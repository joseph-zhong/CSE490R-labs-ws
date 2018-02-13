#!/usr/bin/env python


import cv2
from cv_bridge import CvBridge, CvBridgeError
import math
import matplotlib.pyplot as plt
import numpy as np
from pprint import pprint
import sys
import time


from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped
import tf
from tf import transformations
from sensor_msgs.msg import Image
import rospy

from util import _mask_img, _getDefaultBlobParams_red, DEFAULT_BLOB_PARAMS



blue_boundaries = [
  ((22, 180, 150), (30, 255, 200))
]

red_boundaries = [
  ((122, 235, 110), (129, 255, 200))
]

SLEEP_TIME = 5.0

CAMERA_ANGLE = 0.0
CAMERA_FRAME_PARENT = 'camera_rgb_optical_frame'
CAMERA_FRAME_CHILD = 'base_link'

# For template creation
MAX_ANGLE = 0.40
NUM_TEMPLATES = 50
NUM_PTS = 150
V = 0.46  # Car's current velocity
CAR_LEN = 0.33
IMG_MEDIAN = 268


def average_pt(img):
  rolled_img = np.rollaxis(img, 2, 0)
  rolled_sum = rolled_img[0] + rolled_img[1] + rolled_img[2]
  Y, X = np.where(rolled_sum > 0)

  if len(Y) <= 10:
    return None, None

  X_avg = np.sum(X) / len(X)
  Y_avg = np.sum(Y) / len(Y)

  X_avg = (X_avg - IMG_MEDIAN) * 3 + IMG_MEDIAN

  return X_avg, Y_avg


def create_template(steering):
  """ Uses the Kinematic Model to create a template using 'steering'
    as a constant steering angle."""

  X = []
  Y = []

  last_pos = (0, 0)
  theta = 0
  for i in range(NUM_PTS):
    dt = 0.015
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

  def __init__(self, control_pub, image_pub, params=_getDefaultBlobParams_red()):
    self.cvBridge = CvBridge()
    self.tl = tf.TransformListener()
    self.tb = tf.TransformBroadcaster()
    rospy.sleep(rospy.Duration(SLEEP_TIME))
    self.tl.waitForTransform(CAMERA_FRAME_PARENT, CAMERA_FRAME_CHILD, rospy.Time(), rospy.Duration(5.0))


    # Calculate rotation matrix and translation vector
    translation, rotation = self.tl.lookupTransform(CAMERA_FRAME_PARENT, CAMERA_FRAME_CHILD, rospy.Time())
    # x, y, z = transformations.euler_from_quaternion(rotation)
    # rot_matrix = transformations.euler_matrix(x + CAMERA_ANGLE, y, z)

    rot_matrix = transformations.quaternion_matrix(rotation)

    translation = list(translation) + [1]
    rot_matrix[:, -1] = translation
    rot_matrix = np.array(rot_matrix)
    print(rot_matrix)

    self.control_pub = control_pub
    self.image_pub = image_pub

    # For collecting template data
    template_topic = '/forward_templates'
    self.template_pub = rospy.Publisher(template_topic, Image, queue_size=1)

    templates = []
    self.robot_frames = []
    self.camera_frames = []
    self.pixel_frames = None
    self.point_frames = None
    self.point_frames_heights = None  # For plotting templates for writeup.

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
    # if cv2.__version__.startswith("3."):
      # self.blobDetector = cv2.SimpleBlobDetector_create(params)
    # else:
      # self.blobDetector = cv2.SimpleBlobDetector(params)


  def image_cb(self, msg):
    if self.preprocessed:
      start = time.time()
      predicted_control = self.control  # None at the start

      brg_img = self.cvBridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
      hsv_img = cv2.cvtColor(brg_img, cv2.COLOR_BGR2HSV)

      mask_img = _mask_img(hsv_img, red_boundaries)

      # mask_img = cv2.GaussianBlur(mask_img, (5,5), 5)
      # self.visualize(mask_img, 0,0,0)
      # Compute blobs.
      # keypoints = self.blobDetector.detect(mask_img)
      # self.visualize_key_points(mask_img, keypoints)
      # print keypoints
      # max_keypoint = None
      # for i, keypoint in enumerate(keypoints):
        # if max_keypoint is None or (keypoint.pt[1] < max_keypoint.pt[1] or abs(keypoint.pt[0] - 268) > abs(max_keypoint.pt[0] - 268)):
         # max_keypoint = keypoint

      x, y = average_pt(mask_img)
      if x is not None and y is not None:
        # x, y = max_keypoint.pt[0], max_keypoint.pt[1]

        distances = np.abs(self.point_frames - x)
        min_dist_idx = np.argmin(distances)

        x_p = self.point_frames[min_dist_idx]
        self.visualize(mask_img, x_p, x, y)

        # Templates align with the angles that created them.
        predicted_control = self.discretized_angles[min_dist_idx]
        self.control = predicted_control

      print "CONTROL: ", predicted_control, "TIME", time.time() - start
      if predicted_control is not None:
        self.publish_controls(predicted_control)


  def publish_controls(self, steering_angle):
    ads = AckermannDriveStamped()
    ads.drive.steering_angle = steering_angle
    ads.drive.speed = V
    self.control_pub.publish(ads)


  def visualize(self, image,x_p, x, y):
    # Visualize an image to the object's given image_pub topic.
    if image is not None:
      if y is not None and x is not None:
        x = int(x)
        y = int(y)
        x_p = int(x_p)
        cv2.circle(image, (x, y), 15, (0, 255, 0), 3, cv2.LINE_AA)
        cv2.circle(image, (x_p, y), 15, (255, 0, 0), 3, cv2.LINE_AA)
      rosImg = self.cvBridge.cv2_to_imgmsg(image)
      self.image_pub.publish(rosImg)


  def visualize_key_points(self, img, key_points):
    for key_point in key_points:
      x = int(key_point.pt[0])
      y = int(key_point.pt[1])
      cv2.circle(img, (x, y), 15, (0, 255, 0), 3, cv2.LINE_AA)
    rosImg = self.cvBridge.cv2_to_imgmsg(img)
    self.image_pub.publish(rosImg)


  def k_cb(self, msg):

    if not self.preprocessed:

      k = msg.K
      self.pixel_frames = []
      self.point_frames = []
      self.point_frames_heights = []

      K = np.array(list(k)).reshape(3, 3)

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
        self.point_frames_heights.append(v_f)

      self.point_frames = np.array(self.point_frames)
      self.preprocessed = True  # Callback will now utilize frames


    if self.preprocessed:
      # Publish templates for documentation
      template_img = np.zeros((480, 640, 3), dtype=np.uint8)
      for i, x in enumerate(self.point_frames):
        cv2.circle(template_img, (int(x), int(self.point_frames_heights[i])), 15, (0, 255, 0), 3, cv2.LINE_AA)

      rosImg = self.cvBridge.cv2_to_imgmsg(template_img)
      self.template_pub.publish(rosImg)




