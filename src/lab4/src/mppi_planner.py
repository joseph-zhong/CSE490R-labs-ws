#!/usr/bin/env python

import rospy
import numpy as np
import time

from std_msgs.msg import Bool
import Utils as Utils

from geometry_msgs.msg import PoseStamped
from lab4.srv import *

from nav_msgs.srv import GetPlan

PLANNER_SERVICE_TOPIC = '/planner_node/get_plan'  # The topic at which the service is available

# SOURCE = [-48.68, -0.28, 0]  # Where the plan should start
# TARGET = [1.56, -34.16, 0]  # Where the plan should finish

# SOURCE = [0.28,  8.44, 0.0]
# TARGET = [-44.92,  -18.68,1.57]

# SOURCE = [-8.76,  -2.04, 0.0]
# TARGET = [-10.68,  -24.28, 0.0]

class MPPI_Planner(object):
  def __init__(self):

    self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

    print "Waiting for planning service"
    # rospy.wait_for_service(PLANNER_SERVICE_TOPIC)  # Wait for the service to become available if necessary
    # self.get_plan = rospy.ServiceProxy(PLANNER_SERVICE_TOPIC, GetPlan)  # Setup a ros service client
    print "Planning service is up"

    self.way_point_index = 0

    import pdb
    pdb.set_trace()
    start_pose = Utils.get_csv("/home/tim/Documents/CSE 490/final/start.csv")
    blues_array = Utils.get_csv("/home/tim/Documents/CSE 490/final/good_waypoints.csv")

    self.full_path = self.process_all_blues(start_pose, blues_array)

    self.is_close_sub = rospy.Subscriber("is_close", Bool, self.is_close_cb, queue_size=1)

  # When this is called, we know we made it to our last goal
  def _process_waypoint(self):
    print "Processing the next waypoint"
    next_goal = self.blues[self.way_point_index]
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.pose.position.x = next_goal[0]
    goal_pose.pose.position.y = next_goal[1]
    goal_pose.pose.orientation = Utils.angle_to_quaternion(next_goal[2])
    self.goal_pub.publish(next_goal)


  # def inferred_pose_cb(self, msg):
  #   self.curr_pose = np.array([msg.pose.position.x, msg.pose.position.y, Utils.quaternion_to_angle(msg.pose.orientation)])


  def is_close_cb(self, msg):
    self._process_waypoint()

  # Returns a numpy array of shape (N, 3)
  def process_all_blues(self, start_pose, blues_array):
    pass


if __name__ == '__main__':
  print "[mppi_planner main:] entered main..."

  rospy.init_node('planner_test', anonymous=True)  # Initialize the node

  # Instantiate MPPI Planner.
  mp = MPPI_Planner()



  #
  # x, y = blues[0]
  # target = tuple(x, y, 0)
  # try:
  #   # Get the plan from the service, reshaped as (n, 3)
  #   resp = get_plan(mp.curr_pose, target)
  #   plan = np.array(resp.plan).reshape(-1, 3)
  #
  #   print "[mppi_planner main:] Iterating '{}' subgoals to target '{}'...".format(len(plan), target)
  #   # Publish goal to MPPI.
  #   for goal in plan:
  #     mp.goal_pub.publish(Utils.particle_to_posestamped(target, 'map'))
  #     # mp.goal_cb(goal)
  #
  #     # while not mp.is_close_to_goal(mp.last_pose):
  #     #   print "[mppi_planner main] not close to goal, waiting..."
  #     #   time.sleep(10)
  #
  #     print "resp.plan", plan
  #     print "resp.success", resp.success
  # except rospy.ServiceException, e:
  #   print 'Service call failed: %s' % e
  #
  #
  #
  # # Iterate through the waypoints as targets.
  # print "[mppi_planner main:] Iterating through blue waypoint targets..."
  # # REVIEW josephz: Sort blues in greedily short distance?
  # # for x, y in blues:
