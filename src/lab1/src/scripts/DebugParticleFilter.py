#!/usr/bin/env python
"""
DebugParticleFilter.py
---



"""

import os
import sys

import rospy
import rosbag
from ackermann_msgs.msg import AckermannDriveStamped

# Name of the topic that should be extracted from the bag
BAG_TOPIC = "/scan"
PUB_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'

# The rate at which messages should be published
PUB_RATE = 20

def main(argv):
  args = rospy.myargv()
  if len(args) != 3:
    rospy.loginfo("Args: <bag_path> <follow_backwards as True or False>")
    exit()

  rospy.loginfo("Starting BagFollow with args: '%s'", args)

  bag_path = args[1]
  follow_backwards = True if args[2] == "True" else False
  # follow_backwards = False
  assert os.path.isfile(bag_path), "File not found: '{}'".format(bag_path)

  pub = rospy.Publisher(PUB_TOPIC, AckermannDriveStamped, queue_size=10)
  rospy.init_node('bag_follower', anonymous=True)
  rate = rospy.Rate(PUB_RATE)

  # Populate param(s) with value(s) passed by launch file

  rospy.loginfo("Following bag: '%s'", bag_path)
  bag = rosbag.Bag(bag_path)

  msgs = tuple(bag.read_messages(topics=[BAG_TOPIC]))
  rospy.loginfo("Following bag of '%s' messages", len(msgs))

  if follow_backwards:
    rospy.loginfo("Following backwards...")
    raise NotImplementedError()

  for topic, msg, time in msgs:
    if rospy.is_shutdown():
      break

    rospy.loginfo("Following bag...")
    # print msg.header, msg.drive

    pub.publish(msg)
    rate.sleep()

  rospy.loginfo("Done following bag!")

if __name__ == '__main__':
  main(sys.argv)
