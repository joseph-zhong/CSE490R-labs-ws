#!/usr/bin/env python

import rospy

from Forward import ForwardController
from Feedback import FeedbackController
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Image



FEEDBACK_CONTROLLER = "feedback"
FORWARD_CONTROLLER = "forward"
PUB_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'
SUB_TOPIC = '/camera/color/image_raw'

class Controller(object):
  def __init__(self):
    self.CONTROLLER_TYPE = rospy.get_param("~controller_type", FORWARD_CONTROLLER)
    if self.CONTROLLER_TYPE == FORWARD_CONTROLLER:
      self.controller = ForwardController()
    else:
      self.controller = FeedbackController()


    # Sets up the subscribers and publishers.
    self.control_pub = rospy.Publisher(PUB_TOPIC, AckermannDriveStamped, queue_size=1)  # Publishes the expected pose
    self.image_sub = rospy.Subscriber(SUB_TOPIC, Image, self.controller.image_cb, queue_size=1)


if __name__ == '__main__':
  rospy.init_node('controller', anonymous=True)
  controller = Controller()

  while not rospy.is_shutdown():
    # stuff.
    pass
