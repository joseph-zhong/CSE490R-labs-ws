

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


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

def _getDefaultBlobParams():
  """ Caches default global blob detector params. """
  global DEFAULT_BLOB_PARAMS
  if DEFAULT_BLOB_PARAMS is not None:
    return DEFAULT_BLOB_PARAMS
  else:
    DEFAULT_BLOB_PARAMS = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    DEFAULT_BLOB_PARAMS.minThreshold = 10
    DEFAULT_BLOB_PARAMS.maxThreshold = 10

