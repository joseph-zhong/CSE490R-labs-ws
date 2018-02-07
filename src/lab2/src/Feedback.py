#!/usr/bin/env python

from pprint import pprint
from ackermann_msgs.msg import AckermannDriveStamped
import rospy

SPEED = 0.5
KP = 1
KI = 1
KD = 0

class FeedbackController(object):
    def __init__(self, conrol_pub):
        self.control_pub = conrol_pub
        self.total_error = 0
        self.last_error = 0

    def image_cb(self, msg):
        start = rospy.Time.now()
        error, image = self.img_to_error(msg)
        steering_angle = self.error_to_control(error)
        process_time = rospy.Time.now() - start
        self.visualize(steering_angle, process_time, image)
        self.publish_controls(steering_angle)

    def img_to_error(self, msg):
        image = msg
        return 0, image

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