#!/usr/bin/env python

import rospy
import numpy as np
from lab4.srv import *
from nav_msgs.srv import GetMap


import Utils

import sys
sys.path.append("/home/tim/catkin_ws/src/CSE490R-labs-ws/src/lab3")
PLANNER_SERVICE_TOPIC = '/planner_node/get_plan'  # The topic at which the service is available

# SOURCE = [-48.68, -0.28, 0]  # Where the plan should start
# TARGET = [1.56, -34.16, 0]  # Where the plan should finish

# SOURCE = [0.28,  8.44, 0.0]
# TARGET = [-44.92,  -18.68,1.57]

# SOURCE = [-8.76,  -2.04, 0.0]
# TARGET = [-10.68,  -24.28, 0.0]

SOURCE = [600, 2450, 0.0]
TARGET = [1600, 1700, 0.0]


if __name__ == '__main__':
  print "Entered main"
  rospy.init_node('planner_test', anonymous=True)  # Initialize the node

  map_service_name = rospy.get_param("~static_map", "static_map")
  print("Getting map from service: ", map_service_name)
  rospy.wait_for_service(map_service_name)
  map_info = rospy.ServiceProxy(map_service_name, GetMap)().map.info

  rospy.wait_for_service(PLANNER_SERVICE_TOPIC)  # Wait for the service to become available if necessary
  get_plan = rospy.ServiceProxy(PLANNER_SERVICE_TOPIC, GetPlan)  # Setup a ros service client

  try:
    resp = get_plan(Utils.map_to_world(SOURCE, map_info), Utils.map_to_world(TARGET, map_info))  # Get the plan from the service

    print np.array(resp.plan).reshape(-1, 3)  # Reshape the plan to be nx3
    print resp.success
  except rospy.ServiceException, e:
    print 'Service call failed: %s' % e


