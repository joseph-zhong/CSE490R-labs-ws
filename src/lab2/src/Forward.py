#!/usr/bin/env python


import matplotlib.pyplot as plt
import numpy as np
from pprint import pprint

from ackermann_msgs.msg import AckermannDriveStamped

from util import _mask_img


MAX_ANGLE = 0.34
NUM_TEMPLATES = 20
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
    pprint(msg)


  def visulize(self):
      pass





test = ForwardController("hello")



