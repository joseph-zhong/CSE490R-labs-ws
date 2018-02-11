#!/usr/bin/env python

import cv2
import time
import numpy as np
from pprint import pprint

from cv_bridge import CvBridge, CvBridgeError
from ackermann_msgs.msg import AckermannDriveStamped
import rospy
from util import _mask_img

# Setup Globals.
SPEED = 0.5
KP = 0.0010625
KI = 0
KD = 0
MAX_ANGLE = 0.34

ROI_HEIGHT_HI = 200
ROI_HEIGHT_LO = 10
# HSV triplet boundaries.
# REVIEW josephz: This needs to be tuned, consider instantiating outside this class.
boundaries = [
  ((5, 100, 100), (35, 240, 220))
]

# Setup SimpleBlobDetector parameters.
DEFAULT_BLOB_PARAMS = None
def _getDefaultBlobParams():
  """ Caches default global blob detector params. """
  global DEFAULT_BLOB_PARAMS
  if DEFAULT_BLOB_PARAMS is not None:
    return DEFAULT_BLOB_PARAMS
  else:
    DEFAULT_BLOB_PARAMS = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    DEFAULT_BLOB_PARAMS.minThreshold = 10
    DEFAULT_BLOB_PARAMS.maxThreshold = 100

    # Filter by Area.
    # DEFAULT_BLOB_PARAMS.filterByArea = True
    # DEFAULT_BLOB_PARAMS.minArea = 100

    # Filter by Circularity
    # DEFAULT_BLOB_PARAMS.filterByCircularity = True
    # DEFAULT_BLOB_PARAMS.minCircularity = 0.1

    # Filter by Convexity
    # DEFAULT_BLOB_PARAMS.filterByConvexity = True
    # DEFAULT_BLOB_PARAMS.minConvexity = 0.87

    # Filter by Inertia
    # DEFAULT_BLOB_PARAMS.filterByInertia = True
    # DEFAULT_BLOB_PARAMS.minInertiaRatio = 0.01

class FeedbackController(object):
  def __init__(self, conrol_pub, image_pub, roi_pub=None, params=_getDefaultBlobParams()):
    self.image_pub = image_pub
    self.control_pub = conrol_pub
    self.total_error = 0
    self.last_error = 0

    self.roi_pub = roi_pub

    self.cvBridge = CvBridge()
    # REVIEW josephz: Verify version OpenCV 3.x on the robots.
    if cv2.__version__.startswith("3."):
      self.blobDetector = cv2.SimpleBlobDetector_create(params)
    else:
      self.blobDetector = cv2.SimpleBlobDetector(params)
    self.img_width = None

  def image_cb(self, msg):
    start = rospy.Time.now()
    error, image = self.img_to_error(msg)
    steering_angle = self.error_to_control(error)
    process_time = rospy.Time.now() - start
    #self.visualize(steering_angle, process_time, image)
    self.publish_controls(steering_angle)

  def img_to_error(self, msg):
    s_time = time.time()
    brg_img = self.cvBridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    hsv_img = cv2.cvtColor(brg_img, cv2.COLOR_BGR2HSV)
    mask_img = _mask_img(hsv_img, boundaries)

    img_height, img_width, _ = mask_img.shape
    if self.img_width is None:
      self.img_width = img_width
    roi_lo = ROI_HEIGHT_LO
    roi_hi = min(img_height, ROI_HEIGHT_HI)
    roi_img = mask_img[roi_lo:roi_hi, :, :]


    self.visualize(image=mask_img, roi_img=roi_img)
    src = np.where(roi_img != 0)
    if len(src) == 0 or len(src[0]) == 0:
      print "ERROR MASK IS EMPTY, Cannot average nothing"
      return 0, None

    # For more information on the information provided by keypoints,
    # see https://docs.opencv.org/2.4/modules/features2d/doc/common_interfaces_of_feature_detectors.html#Point2f
    # %20pt
    # In particular, here we are only using xy-coordinate pairs from `pt`,
    # but we could also take into account `size, and especially `angle` and `response`.
    keypoints = self.blobDetector.detect(roi_img)
    max_keypoint = None
    for i, keypoint in enumerate(keypoints):
      if max_keypoint is None or keypoint.size > max_keypoint.size:
        max_keypoint = keypoint
    if max_keypoint is None:
      print "MAX_KEYPOINT IS NONE"
      return 0
    print "[img_width / 2 : {}] [center of blob: {}]".format(float(img_width) / 2, max_keypoint.pt[0])
    err = float(img_width) / 2 - max_keypoint.pt[0]

    print "[Error: {}] [Image Shape: {}] [ROI Shape: {}] [Compute Time: {}]".format(
        err, mask_img.shape, roi_img.shape, time.time() - s_time)
    return err, mask_img

  def error_to_control(self, error):
    delta_error = error - self.last_error
    pprint(error)
    steering_angle = (KP * error) + (KI * self.total_error) + (KD * delta_error)
    print("steering angle")
    pprint(steering_angle)
    self.last_error = error
    self.total_error += error
    return steering_angle

  def publish_controls(self, steering_angle):
    print("moving")
    ads = AckermannDriveStamped()
    ads.drive.steering_angle = steering_angle
    ads.drive.speed = SPEED
    self.control_pub.publish(ads)

  def visualize(self, steering_angle=None, process_time=None, image=None, roi_img=None):
    if image is not None:
      rosImg = self.cvBridge.cv2_to_imgmsg(image)
      self.image_pub.publish(rosImg)

    if roi_img is not None and self.roi_pub is not None:
      ros_roi_img = self.cvBridge.cv2_to_imgmsg(roi_img)
      self.roi_pub.publish(ros_roi_img)


