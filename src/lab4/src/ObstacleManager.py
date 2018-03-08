#!/usr/bin/env python

import cv2
import math
import numpy
import IPython
import Dubins
import Utils
import KinematicModel as model

import matplotlib.pyplot as plt
import numpy as np

import rospy
from nav_msgs.srv import GetMap, GetPlan


class ObstacleManager(object):

  def __init__(self, mapMsg):
    # Setup the map
    self.map_info = mapMsg.info
    self.mapImageGS = numpy.array(mapMsg.data, dtype=numpy.uint8).reshape((mapMsg.info.height, mapMsg.info.width,1))

    # Retrieve the map dimensions
    height, width, channels = self.mapImageGS.shape
    self.mapHeight = height
    self.mapWidth = width
    self.mapChannels = channels

    # Binarize the Image
    self.mapImageBW = 255*numpy.ones_like(self.mapImageGS, dtype=numpy.uint8)
    self.mapImageBW[self.mapImageGS==0] = 0
    self.mapImageBW = self.mapImageBW[::-1,:,:] # Need to flip across the y-axis
    print "self.mapImageBW.shape:", self.mapImageBW.shape

    # Obtain the car length and width in pixels
    self.robotWidth = int(model.CAR_WIDTH/self.map_info.resolution + 0.5)
    self.robotLength = int(model.CAR_LENGTH/self.map_info.resolution + 0.5)

  def get_state_validity(self, config):
    """ Check if the passed config is in collision
    config: The configuration to check (in meters and radians)
    Returns False if in collision, True if not in collision """

    # Convert the configuration to map-coordinates -> mapConfig is in pixel-space
    mapConfig = Utils.world_to_map(config, self.map_info)

    # ---------------------------------------------------------
    # TODO: YOUR CODE HERE
    #
    # Return true or false based on whether the configuration is in collision
    # Use self.robotWidth and self.robotLength to represent the size of the robot
    # Also return false if the robot is out of bounds of the map
    # Although our configuration includes rotation, assume that the
    # rectangle representing the robot is always aligned with the coordinate axes of the
    # map for simplicity
    # ----------------------------------------------------------

    print "mapConfig:", mapConfig
    assert isinstance(mapConfig, (list, tuple)) and len(mapConfig) == 3
    y, x, theta = mapConfig
    halfLen = self.robotLength / 2
    halfWidth = self.robotWidth / 2
    # REVIEW josephz: Figure out how to incorporate theta to differentiate the two cases?
    # Validity for car length along the map length.
    valid = self.mapImageBW[
            max(0, y-halfLen):min(self.mapHeight, y+halfLen),
            max(0, x-halfWidth):min(self.mapWidth, x+halfWidth)]

    # Validity for car length along the map width.
    valid2 = self.mapImageBW[
            max(0, y - halfWidth):min(self.mapHeight, y + halfWidth),
            max(0, x - halfLen):min(self.mapWidth, x + halfLen)]

    plt.figure()
    valid[:] = 50
    valid2[:] = 100
    plt.imshow(np.squeeze(valid))
    plt.imshow(np.squeeze(valid2))
    plt.imshow(np.squeeze(self.mapImageBW))
    # plt.show()
    return np.sum(valid) == np.sum(valid2) == 0

  # Check if there is an unobstructed edge between the passed configs
  # config1, config2: The configurations to check (in meters and radians)
  # Returns false if obstructed edge, True otherwise
  def get_edge_validity(self, config1, config2):
    # -----------------------------------------------------------
    # TODO: YOUR CODE HERE
    #
    # Check if endpoints are obstructed, if either is, return false
    # Find path between two configs using Dubins
    # Check if all configurations along Dubins path are obstructed 
    # -----------------------------------------------------------

    # (Tam: Suggested implementation)
    # TODO: config1 and config2 are src and dest. respectively, which
    # will have intermediate waypoints between them. To detect if the
    # path between them is valid
    # 1. for each item in  [config1, intermed1, intermed2, ... intermedn, config2]:
    #        if not get_state_validity(item):
    #            return False
    #    return True

    # REVIEW josephz: Is endpoint checking incorporated within path?
    if not self.get_state_validity(config1): return False
    if not self.get_state_validity(config2): return False
    path = Dubins.dubins_path_planning(config1, config2, 1.0 / model.TURNING_RADIUS)
    plt.show()
    return all(self.get_state_validity(config) for config in path)

# Test
if __name__ == '__main__':
  print "Starting obstacle manager"
  rospy.init_node("obstacle_manager_test", anonymous=True)
  bounds1 = [-22.3266181946, -10.6194477081, 0]
  bounds2 = [2, 3, 1]

  map_service_name = rospy.get_param("~static_map", "static_map")
  print("Getting map from service: ", map_service_name)
  rospy.wait_for_service(map_service_name)
  map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map

  obs_manager = ObstacleManager(map_msg)

  print "Bounds 1:", bounds1
  bounds1_map = Utils.world_to_map(bounds1, obs_manager.map_info)
  print "Bounds 1 in the map is ", bounds1_map
  bounds1_world = Utils.map_to_world(bounds1_map, obs_manager.map_info)
  print "Bound 1 back in the world are ", bounds1_world

  bound1_valid = obs_manager.get_state_validity(bounds1)
  print "Is bounds1 valid?", bound1_valid


  # Tim can write this test
  # Write test code here!