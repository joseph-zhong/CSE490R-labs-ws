

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


BOUNDARIES = [
    ((5, 100, 100), (35, 240, 220))
]


def _mask_img(img):
    """ Applies the boundaries to produce the mask image.
    """
    img_out = np.zeros_like(img)
    for (lower, upper) in BOUNDARIES:
        lower = np.array(lower)
        upper = np.array(upper)

        mask = cv2.inRange(img, lower, upper)
        cv2.bitwise_and(img, img, mask=mask, dst=img_out)

    return img_out

