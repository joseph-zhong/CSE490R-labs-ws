#!/usr/bin/env python

import rospy

from Forward import ForwardController
from Feedback import FeedbackController

FEEDBACK_CONTROLLER = "feedback"
FORWARD_CONTROLLER = "forward"

class Controller(object):
  def __init__(self):
    self.CONTROLLER_TYPE = rospy.get_param("~controller_type", FORWARD_CONTROLLER)
    if self.CONTROLLER_TYPE == FORWARD_CONTROLLER:
      self.controller = ForwardController()
    else:
      self.controller = FeedbackController()

    # Setup the subscribers and publishers.


if __name__ == '__main__':
  controller = Controller

  while not rospy.is_shutdown():
    # stuff.
    pass
