#!/usr/bin/env python
import csv

import rospy
import numpy as np
import time
from lab4.srv import *
# from lab3.src import *
# import lab3
import MPPI

from nav_msgs.srv import GetPlan

PLANNER_SERVICE_TOPIC = '/planner_node/get_plan'  # The topic at which the service is available

SOURCE = [-48.68, -0.28, 0]  # Where the plan should start
TARGET = [1.56, -34.16, 0]  # Where the plan should finish

# SOURCE = [0.28,  8.44, 0.0]
# TARGET = [-44.92,  -18.68,1.57]

# SOURCE = [-8.76,  -2.04, 0.0]
# TARGET = [-10.68,  -24.28, 0.0]

if __name__ == '__main__':
  print "[mppi_planner main:] entered main..."

  import pdb
  pdb.set_trace()
  rospy.init_node('planner_test', anonymous=True)  # Initialize the node
  rospy.wait_for_service(PLANNER_SERVICE_TOPIC)  # Wait for the service to become available if necessary
  get_plan = rospy.ServiceProxy(PLANNER_SERVICE_TOPIC, GetPlan)  # Setup a ros service client

  # Set up MPPI Controller.
  mp = MPPI.MPPIController()

  # Initialize source and target.
  source = mp.last_pose

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

  # Iterate through the waypoints as targets.
  print "[mppi_planner main:] Iterating through blue waypoint targets..."
  for x, y in blues:
    target = tuple(x, y, 0)
    try:
      # Get the plan from the service, reshaped as (n, 3)
      resp = get_plan(source, target)
      plan = np.array(resp.plan).reshape(-1, 3)

      print "[mppi_planner main:] Iterating '{}' subgoals to target '{}'...".format(len(plan), target)
      for goal in plan:
        mp.goal_cb(goal)

        while not mp.is_close_to_goal(mp.last_pose):
          print "[mppi_planner main] not close to goal, waiting..."
          time.sleep(10)

        print "resp.plan", plan
        print "resp.success", resp.success
    except rospy.ServiceException, e:
      print 'Service call failed: %s' % e

