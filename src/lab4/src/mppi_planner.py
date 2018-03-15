#!/usr/bin/env python

import os
import rospy
import numpy as np
import time

from std_msgs.msg import Bool
import Utils as Utils
import UtilsTh as UtilsTh
from nav_msgs.srv import GetMap
from nav_msgs.msg import Path



from geometry_msgs.msg import PoseStamped, PoseArray, Point
from lab4.srv import *

PLANNER_SERVICE_TOPIC = '/planner_node/get_plan'  # The topic at which the service is available

# SOURCE = [-48.68, -0.28, 0]  # Where the plan should start
# TARGET = [1.56, -34.16, 0]  # Where the plan should finish

# SOURCE = [0.28,  8.44, 0.0]
# TARGET = [-44.92,  -18.68,1.57]

# SOURCE = [-8.76,  -2.04, 0.0]
# TARGET = [-10.68,  -24.28, 0.0]

# START_POSE_FNAME = "/home/tim/catkin_ws/src/CSE490R-labs-ws/src/lab4/received/start.csv"
# GOOD_WAYPOINTS_FNAME = "/home/tim/catkin_ws/src/CSE490R-labs-ws/src/lab4/received/good_waypoints.csv"

START_POSE_FNAME = "received/start.csv"
GOOD_WAYPOINTS_FNAME = "received/good_waypoints.csv"

DUBINS_PATH_FNAME = 'paths/full_dubins_path.npy'
STEP_NUMBER = 1

class MPPI_Planner(object):
  def __init__(self):

    self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    self.path_pub = rospy.Publisher("waypoints", Path, queue_size=20)
    self.pose_array_pub = rospy.Publisher("poses", PoseArray, queue_size=40)
    self.blue_waypoints_pub = rospy.Publisher("blue_waypoints", Point, queue_size=5)

    map_service_name = rospy.get_param("~static_map", "static_map")
    print("Getting map from service: ", map_service_name)
    rospy.wait_for_service(map_service_name)
    map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map  # The map, will get passed to init of sensor model
    self.map_info = map_msg.info  # Save info about map for later use

    print "Waiting for planning service"
    rospy.wait_for_service(PLANNER_SERVICE_TOPIC)  # Wait for the service to become available if necessary
    self.get_plan = rospy.ServiceProxy(PLANNER_SERVICE_TOPIC, GetPlan)  # Setup a ros service client
    print "Planning service is up"

    self.way_point_index = 0

    print "Reading starting pose and blue_array..."
    assert os.path.isfile(START_POSE_FNAME), "{} not found, make sure to run from a dir relative to that file, try `roscd lab4`".format(START_POSE_FNAME)
    assert os.path.isfile(GOOD_WAYPOINTS_FNAME), "{} not found, make sure to run from a dir relative to that file, try `roscd lab4`".format(GOOD_WAYPOINTS_FNAME)
    assert os.path.isdir('paths'), "paths directory not found, try `roscd lab4`"

    start_pose = Utils.csv_to_configs(START_POSE_FNAME)
    blues_array = Utils.csv_to_configs(GOOD_WAYPOINTS_FNAME)




    self.full_path = self.process_all_blues(start_pose, blues_array)

    np.save('paths/full_dubins_path.npy', self.full_path)

  # Returns a numpy array of shape (N, 3)
  def process_all_blues(self, start_pose, blues_array):
    # For visualization
    pa = Path()
    pose_array = PoseArray()
    frame_id = 'map'
    pa.header = UtilsTh.make_header(frame_id)
    pose_array.header = UtilsTh.make_header(frame_id)


    print "[mppi_planner process_all_blues:] entering process_all_blues..."
    all_poses = np.concatenate((start_pose, blues_array), axis=0)

    print "[mppi_planner process_all_blues:] All Poses:", all_poses

    len_all_poses = all_poses.shape[0]
    resp = np.empty((0, 3))

    # import pdb
    # pdb.set_trace()
    for i in range(1, len_all_poses):
      print "[mppi_planner process_all_blues: loop:] i:", i
      if i == len_all_poses - 1:
        theta_curr = 0
      else:
        theta_curr = Utils.angle_between_points(all_poses[i], all_poses[i + 1])

      theta_prev = Utils.angle_between_points(all_poses[i - 1], all_poses[i])
      print "[mppi_planner process_all_blues:] Theta prev:", theta_prev
      prev = Utils.map_to_world(np.append(all_poses[i - 1], theta_prev), self.map_info)
      curr = Utils.map_to_world(np.append(all_poses[i], theta_curr), self.map_info)
      new_resp = self.get_plan(prev, curr)
      print "[mppi_planner process_all_blues:] new_resp:", new_resp
      new_resp = np.array(new_resp.plan).reshape(-1, 3)

      # Interpolate the path.
      pruned_new_resp = [prev]
      for j in xrange(len(new_resp) - 1):
        a = pruned_new_resp[-1]
        b = new_resp[j+1]
        if _isFar(a, b):
          pruned_new_resp.append(b)
      pruned_new_resp[-1] = curr

      resp = np.append(resp, np.array(pruned_new_resp), axis=0)
      pa.poses = map(UtilsTh.particle_to_posestamped, resp, [frame_id] * len(resp))
      pose_array.poses = map(UtilsTh.particle_to_pose, resp)
      self.path_pub.publish(pa)
      self.pose_array_pub.publish(pose_array)


    print "[mppi_planner process_all_blues:] resp:", resp
    return resp


def _isFar(a, b):
  """ Configs are [x, y, theta] configurations in world space."""
  import MPPI
  diff = np.abs(a - b)
  angle_between = min(diff[2], np.pi * 2 - diff[2])
  is_far = diff[0] >= MPPI.DIST_THRES / 2.0 or diff[1] >= MPPI.DIST_THRES / 2.0 \
           or angle_between >= MPPI.THETA_THRES
  # print "exiting is_far_to_goal, diff:", diff, "angle_between:", angle_between, "is far?", is_far
  return is_far

if __name__ == '__main__':
  print "[mppi_planner main:] entered main..."

  rospy.init_node('planner_test', anonymous=True)  # Initialize the node

  # Instantiate MPPI Planner.
  mp = MPPI_Planner()



# TODO: Smart Blues Sorting?
