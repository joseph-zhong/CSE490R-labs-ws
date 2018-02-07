#!/usr/bin/env python

import cv2
import numpy as np
from pprint import pprint

from cv_bridge import CvBridge, CvBridgeError
from ackermann_msgs.msg import AckermannDriveStamped
import rospy

# Setup Globals.
SPEED = 0.5
KP = 1
KI = 1
KD = 0

ROI_HEIGHT_HI = 200
ROI_HEIGHT_LO = 10
# HSV triplet boundaries.
# REVIEW josephz: This needs to be tuned, consider instantiating outside this class.
BOUNDARIES = [
  ((217, 50, 50), (217, 100, 100))
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
        # DEFAULT_BLOB_PARAMS.minArea = 1500

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
    def __init__(self, conrol_pub, params=_getDefaultBlobParams()):
        self.control_pub = conrol_pub
        self.total_error = 0
        self.last_error = 0

        self.cvBridge = CvBridge()
        # REVIEW josephz: Verify version OpenCV 3.x on the robots.
        if cv2.__version__.startswith("3."):
            self.blobDetector = cv2.SimpleBlobDetector_create(params)
        else:
            self.blobDetector = cv2.SimpleBlobDetector(params)

    def image_cb(self, msg):
        start = rospy.Time.now()
        error, image = self.img_to_error(msg)
        steering_angle = self.error_to_control(error)
        process_time = rospy.Time.now() - start
        self.visualize(steering_angle, process_time, image)
        self.publish_controls(steering_angle)

    def img_to_error(self, msg):
        brg_img = self.cvBridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        hsv_img = cv2.cvtColor(brg_img, cv2.COLOR_BGR2HSV)
        out_img = self._mask_img(hsv_img)

        img_height, img_width, _ = out_img.shape
        roi_lo = ROI_HEIGHT_LO
        roi_hi = max(img_height, ROI_HEIGHT_HI)
        roi_img = out_img[roi_lo:roi_hi, :, :]

        # For more information on the information provided by keypoints,
        # see https://docs.opencv.org/2.4/modules/features2d/doc/common_interfaces_of_feature_detectors.html#Point2f
        # %20pt
        # In particular, here we are only using xy-coordinate pairs from `pt`,
        # but we could also take into account `size, and especially `angle` and `response`.
        keypoints = self.blobDetector.detect(roi_img)
        avg_x = np.average([keypoint.pt[0] for keypoint in keypoints], axis=0)

        err = img_width / 2 - avg_x
        return err

    def _mask_img(self, img):
        """ Applies the boundaries to produce the mask image.
        """
        img_out = np.zeros_like(img)
        for (lower, upper) in BOUNDARIES:
            mask = cv2.inRange(img, lower, upper)
            cv2.bitwise_and(img, img, mask=mask, dst=img_out)
        return img_out

    def error_to_control(self, error):
        delta_error = error - self.last_error
        steering_angle = (KP * error) + (KI * self.total_error) + (KD * delta_error)
        self.last_error = error
        self.total_error += error
        return steering_angle

    def publish_controls(self, steering_angle):
        print("moving")
        ads = AckermannDriveStamped()
        ads.drive.steering_angle = steering_angle
        ads.drive.speed = SPEED
        self.control_pub.publish(ads)

    def visualize(self, steering_angle, process_time, image):
        pass
