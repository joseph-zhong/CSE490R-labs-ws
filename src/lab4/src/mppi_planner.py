#!/usr/bin/env python
import csv

import rospy
import numpy as np
import time

from std_msgs.msg import Bool
import utils as Utils

from geometry_msgs.msg import PoseStamped
from lab4.srv import *

from nav_msgs.srv import GetPlan

PLANNER_SERVICE_TOPIC = '/planner_node/get_plan'  # The topic at which the service is available

SOURCE = [-48.68, -0.28, 0]  # Where the plan should start
TARGET = [1.56, -34.16, 0]  # Where the plan should finish

# SOURCE = [0.28,  8.44, 0.0]
# TARGET = [-44.92,  -18.68,1.57]

# SOURCE = [-8.76,  -2.04, 0.0]
# TARGET = [-10.68,  -24.28, 0.0]

class MPPI_Planner(object):
  def __init__(self, blues, reds):
    self.pose_sub = rospy.Subscriber("/pf/viz/inferred_pose", PoseStamped, self.inferred_pose_cb, queue_size=1)
    self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    self.is_close_sub = rospy.Subscriber("is_close", Bool, self.is_close_cb, queue_size=1)

    self.curr_pose = None
    self.blues = blues
    self.reds = reds
    self.way_point_index = 0


  def _process_waypoint(self):



def inferred_pose_cb(self, msg):
    self.curr_pose = np.array([msg.pose.position.x,
                                 msg.pose.position.y,
                                 Utils.quaternion_to_angle(msg.pose.orientation)])

  def is_close_cb(self, msg):
    self._process_waypoint()

if __name__ == '__main__':
  print "[mppi_planner main:] entered main..."

  rospy.init_node('planner_test', anonymous=True)  # Initialize the node
  rospy.wait_for_service(PLANNER_SERVICE_TOPIC)  # Wait for the service to become available if necessary
  get_plan = rospy.ServiceProxy(PLANNER_SERVICE_TOPIC, GetPlan)  # Setup a ros service client

  # Process data.
  # Read csv for waypoints.
  print "[mppi_planner main:] Reading CSV for waypoints..."
  with open('dummy.csv', 'r') as fin:
    csvReader = csv.reader(fin, delimiter=',')
    way_points = tuple(row for row in csvReader)
    blues = way_points[0]
    reds = way_points[1]
  # REVIEW josephz: The delimiter between the x-y coordinates must be determined.
  blues = tuple((pt.split(';')) for pt in blues)
  reds = tuple((pt.split(';')) for pt in reds)
  print "[mppi_planner main:] Successfully read CSV for '{}' blue and '{}' red waypoints".format(
    len(blues), len(reds))

  # Instantiate MPPI Planner.
  mp = MPPI_Planner(blues, reds)

  x, y = blues[0]
  target = tuple(x, y, 0)
  try:
    # Get the plan from the service, reshaped as (n, 3)
    resp = get_plan(mp.curr_pose, target)
    plan = np.array(resp.plan).reshape(-1, 3)

    print "[mppi_planner main:] Iterating '{}' subgoals to target '{}'...".format(len(plan), target)
    # Publish goal to MPPI.
    for goal in plan:
      mp.goal_pub.publish(Utils.particle_to_posestamped(target, 'map'))
      # mp.goal_cb(goal)

      # while not mp.is_close_to_goal(mp.last_pose):
      #   print "[mppi_planner main] not close to goal, waiting..."
      #   time.sleep(10)

      print "resp.plan", plan
      print "resp.success", resp.success
  except rospy.ServiceException, e:
    print 'Service call failed: %s' % e



  # Iterate through the waypoints as targets.
  print "[mppi_planner main:] Iterating through blue waypoint targets..."
  # REVIEW josephz: Sort blues in greedily short distance?
  # for x, y in blues:
