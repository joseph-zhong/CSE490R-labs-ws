

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from pprint import pprint


def _mask_img(img, boundaries):
  """ Applies the boundaries to produce the mask image.
  """
  img_out = np.zeros_like(img)
  for (lower, upper) in boundaries:
    lower = np.array(lower)
    upper = np.array(upper)

    mask = cv2.inRange(img, lower, upper)
    cv2.bitwise_and(img, img, mask=mask, dst=img_out)

  return img_out


# Setup SimpleBlobDetector parameters.
DEFAULT_BLOB_PARAMS = None

def _getDefaultBlobParams_red():
  """ Caches default global blob detector params. """
  global DEFAULT_BLOB_PARAMS

  if DEFAULT_BLOB_PARAMS is None:
    print("Using custom blob params")
    DEFAULT_BLOB_PARAMS = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    DEFAULT_BLOB_PARAMS.minThreshold = 10
    DEFAULT_BLOB_PARAMS.maxThreshold = 255

  DEFAULT_BLOB_PARAMS.filterByCircularity = False
  DEFAULT_BLOB_PARAMS.filterByConvexity = False
  DEFAULT_BLOB_PARAMS.filterByArea = False
  DEFAULT_BLOB_PARAMS.filterByColor = False
  DEFAULT_BLOB_PARAMS.filterByInertia = False
  return DEFAULT_BLOB_PARAMS

def _getDefaultBlobParams_blue():
  """ Caches default global blob detector params. """
  global DEFAULT_BLOB_PARAMS

  if DEFAULT_BLOB_PARAMS is None:
    print("Using custom blob params")
    DEFAULT_BLOB_PARAMS = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    DEFAULT_BLOB_PARAMS.minThreshold = 10
    DEFAULT_BLOB_PARAMS.maxThreshold = 255

  return DEFAULT_BLOB_PARAMS

def get_blob_key_points():
  pass