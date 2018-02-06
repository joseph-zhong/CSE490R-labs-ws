#!/usr/bin/env python

from pprint import pprint
from ackermann_msgs.msg import AckermannDriveStamped

SPEED = 0.5

class FeedbackController(object):
    def __init__(self, conrol_pub):
        self.control_pub = conrol_pub

    def image_cb(self, msg):
        error = self.img_to_error(msg)
        steering_angle = self.error_to_control(error)
        self.publish_controls(steering_angle)

    def img_to_error(self, msg):
        return 0

    def error_to_control(self, error):
        return 0

    def publish_controls(self, steering_angle):
        print("moving")
        ads = AckermannDriveStamped()
        ads.drive.steering_angle = steering_angle
        ads.drive.speed = SPEED
        self.control_pub.publish(ads)