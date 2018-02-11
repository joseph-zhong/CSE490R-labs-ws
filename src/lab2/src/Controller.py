#!/usr/bin/env python

import rospy

from Forward import ForwardController
from Feedback import FeedbackController
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Image, CameraInfo



FEEDBACK_CONTROLLER = "feedback"
FORWARD_CONTROLLER = "forward"

PUB_CTRL_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'
PUB_IMG_TOPIC = '/mask_img'
PUB_IMG_ROI_TOPIC = '/roi_img'
PUB_IMG_TEMPLATE_TOPIC = '/template_img'

SUB_TOPIC = '/camera/color/image_raw'
SUB_TOPIC_K = '/camera/color/camera_info'

class Controller(object):
  def __init__(self):
    # Publishes the expected pose
    control_pub = rospy.Publisher(PUB_CTRL_TOPIC, AckermannDriveStamped, queue_size=1)
    image_pub = rospy.Publisher(PUB_IMG_TOPIC, Image, queue_size=1)
    roi_pub = rospy.Publisher(PUB_IMG_ROI_TOPIC, Image, queue_size=1)
    template_pub = rospy.Publisher(PUB_IMG_TEMPLATE_TOPIC, Image, queue_size=1)

    self.CONTROLLER_TYPE = rospy.get_param("~controller_type", FORWARD_CONTROLLER)
    if self.CONTROLLER_TYPE == FORWARD_CONTROLLER:
      self.controller = ForwardController(control_pub, template_pub, roi_pub)
      self.k_sub = rospy.Subscriber(SUB_TOPIC_K, CameraInfo, self.controller.k_cb, queue_size=1)
    elif self.CONTROLLER_TYPE == FEEDBACK_CONTROLLER:
      self.controller = FeedbackController(control_pub, image_pub, roi_pub)
    else:
      assert False, "Unrecognized controller type: '{}'".format(self.CONTROLLER_TYPE)

    # Sets up the subscribers and publishers.
    self.image_sub = rospy.Subscriber(SUB_TOPIC, Image, self.controller.image_cb, queue_size=1)


if __name__ == '__main__':
  rospy.init_node('controller', anonymous=True)
  controller = Controller()

  while not rospy.is_shutdown():
    # stuff.
    pass
