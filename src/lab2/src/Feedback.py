#!/usr/bin/env python

import cv2
import time
import numpy as np
from pprint import pprint

from cv_bridge import CvBridge, CvBridgeError
from ackermann_msgs.msg import AckermannDriveStamped
import rospy
from util import _mask_img, _getDefaultBlobParams_blue, DEFAULT_BLOB_PARAMS

# Setup Globals.
SPEED = 0.45
KP = 0.0010625
KI = 0
KD = 0
MAX_ANGLE = 0.34
CENTER_OFFSET = 268

#ROI_HEIGHT_HI = 600
#ROI_HEIGHT_LO = 400
ROI_THICKNESS = 150
# HSV triplet boundaries.
# REVIEW josephz: This needs to be tuned, consider instantiating outside this class.
blue_boundaries = [
  ((22, 180, 150), (30, 255, 200))
]

class FeedbackController(object):
  def __init__(self, conrol_pub, image_pub, roi_pub=None, params=_getDefaultBlobParams_blue()):
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
    """
    Receives image from camera, processes and computes error, and publishes controls.
    """
    # start = rospy.Time.now()
    s_time = time.time()

    # Process image and compute error.
    error, image, roi_img, y = self.img_to_error(msg)
    print "The error we are getting is", error
    # Compute steering angle from error.
    steering_angle = self.error_to_control(error)
    # process_time = rospy.Time.now().nsecs - start.nsecs
    process_time = time.time() - s_time
    #self.visualize(steering_angle, process_time, image=image, roi_img=roi_img, y=y, error=error)
    self.publish_controls(steering_angle)
    # print
    # print "-----------"
    # print "TOTAL PIPELINE TIME: ", time.time() - s_time
    # print "-----------"
    # print

  def img_to_error(self, msg):
    brg_img = self.cvBridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    hsv_img = cv2.cvtColor(brg_img, cv2.COLOR_BGR2HSV)
    mask_img = _mask_img(hsv_img, blue_boundaries)

    print "The shape of the mask image is", mask_img.shape
    img_height, img_width, _ = mask_img.shape
    print "The image height is ", img_height, "and the image width is ", img_width
    if self.img_width is None:
      self.img_width = img_width

    # create the cropped image
    roi_hi = img_height
    roi_lo = img_height - ROI_THICKNESS
    roi_img = mask_img[roi_lo:roi_hi, :, :]
    src = np.where(roi_img != 0)
    if len(src) == 0 or len(src[0]) == 0:
      print "ERROR MASK IS EMPTY, Cannot average nothing"
      return self.last_error, None, None, None


    # For more information on the information provided by keypoints,
    # see https://docs.opencv.org/2.4/modules/features2d/doc/common_interfaces_of_feature_detectors.html#Point2f
    # %20pt
    # In particular, here we are only using xy-coordinate pairs from `pt`,
    # but we could also take into account `size, and especially `angle` and `response`.
    keypoints = self.blobDetector.detect(roi_img)
    self.visualize_key_points(roi_img, keypoints)

    print "len(keypoints): '{}'".format(len(keypoints))
    max_keypoint = None
    for i, keypoint in enumerate(keypoints):
      if max_keypoint is None or keypoint.size > max_keypoint.size:
        max_keypoint = keypoint
    if max_keypoint is None:
      print "MAX_KEYPOINT IS NONE"
      return self.last_error, None, None, None
    err = CENTER_OFFSET - max_keypoint.pt[0]
    y = max_keypoint.pt[1]

    print "[Error: {}] [Image Shape: {}] [ROI Shape: {}]".format(
        err, mask_img.shape, roi_img.shape)
    return err, mask_img, roi_img, y

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

  def visualize(self, steering_angle=None, process_time=None, image=None, roi_img=None, y=None, error=0):
    if image is not None:
      if steering_angle is not None:
        cv2.putText(image, "Steering Angle: '{}'".format(steering_angle),
            (230, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
      if process_time is not None:
        cv2.putText(image, "Processing Time: '{}'".format(process_time),
            (230, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
      if y is not None:
        error = int(error)
        y = int(y)
        cv2.circle(image,(error, y), 15, (0, 255, 0), 3, cv2.LINE_AA)
      rosImg = self.cvBridge.cv2_to_imgmsg(image)
      self.image_pub.publish(rosImg)

    if roi_img is not None and self.roi_pub is not None:
      if steering_angle is not None:
        cv2.putText(roi_img, "Steering Angle: '{}'".format(steering_angle),
          (230, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
      if process_time is not None:
        cv2.putText(roi_img, "Processing Time: '{}'".format(process_time),
            (230, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
      ros_roi_img = self.cvBridge.cv2_to_imgmsg(roi_img)
      # self.image_pub.publish(ros_roi_img)

  def visualize_key_points(self, img, key_points):
    for key_point in key_points:
      x = int(key_point.pt[0])
      y = int(key_point.pt[1])
      cv2.circle(img, (x, y), 15, (255, 0, 0), 3, cv2.LINE_AA)
    rosImg = self.cvBridge.cv2_to_imgmsg(img)
    self.image_pub.publish(rosImg)


