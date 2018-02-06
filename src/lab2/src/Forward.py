#!/usr/bin/env python

from pprint import pprint
from ackermann_msgs.msg import AckermannDriveStamped


class ForwardController(object):
    def __init__(self, control_pub):
        self.control_pub = control_pub

    def image_cb(self, msg):
        pprint(msg)