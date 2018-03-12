#!/usr/bin/env python

import os
import rospy
import numpy as np
import time

from std_msgs.msg import Bool
import Utils as Utils
from nav_msgs.srv import GetMap


from geometry_msgs.msg import PoseStamped
from lab4.srv import *


if __name__ == '__main__':
  print "[mppi_planner main:] entered main..."


  rospy.init_node('map_flipper', anonymous=True)  # Initialize the node

  map_service_name = rospy.get_param("~static_map", "static_map")
  print("Getting map from service: ", map_service_name)
  rospy.wait_for_service(map_service_name)
  map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map  # The map, will get passed to init of sensor model
  map_info = map_msg.info  # Save info about map for later use
  map_height = map_info.height
  file_name = 'full_dubins_path_10_300_obstructed_swap_beg.npy'
  path_array  = np.load(file_name)

  print "Path array before", path_array
  for i in range(path_array.shape[0]):
    path_array[i][1] *= -1
    path_array[i][1] += map_height

  print "Path array after", path_array
  np.save("full_dubins_path_10_300_obstructed_swap_beg_flipped2.npy", path_array)

# TODO: Smart Blues Sorting?
