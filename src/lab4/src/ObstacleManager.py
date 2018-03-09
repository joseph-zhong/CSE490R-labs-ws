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
    self.mapImageBW = self.mapImageBW # [::-1, :, :] # Need to flip across the y-axis
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

    assert isinstance(mapConfig, (list, tuple)) and len(mapConfig) == 3
    x, y, theta = mapConfig
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
    # plt.figure()
    # valid[:] = 50
    # valid2[:] = 100
    # plt.imshow(np.squeeze(self.mapImageBW))
    # plt.imshow(np.squeeze(a))
    # plt.imshow(np.squeeze(b))
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
    path = Dubins.dubins_path_planning(config1, config2, 1.0 / model.TURNING_RADIUS)
    plt.show()
    return all(self.get_state_validity(config) for config in path)

# Test
if __name__ == '__main__':
  print "Starting obstacle manager"
  rospy.init_node("obstacle_manager_test", anonymous=True)

  # Get the map
  map_service_name = rospy.get_param("~static_map", "static_map")
  print("Getting map from service: ", map_service_name)
  rospy.wait_for_service(map_service_name)
  map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map

  # Create the obstacle manager
  obs_manager = ObstacleManager(map_msg)

  print "Testing middle of the hall way"
  print "In hall with skinny triangle: Should be True...", obs_manager.get_state_validity([0.62338411808, -0.209855854511, 0.0])
  print "Skinny triangle: Should be False...", obs_manager.get_state_validity([-0.226983860135, 1.16524136066, 0.0])
  print "Next to chair: Should be False...", obs_manager.get_state_validity([2.71258091927, 2.31397628784, 0.0])
  print "Inside chair: Should be False...", obs_manager.get_state_validity([2.07119321823, 3.0112221241, 0.0])

  print "Testing inside square"
  print "Upper Left corner inside: Should be False...", obs_manager.get_state_validity([10.3908414841, 14.4619655609, 0.0])
  print "Bottom Left corner inside: Should be False...", obs_manager.get_state_validity([3.79972362518, 8.73226356506, 0.0])
  print "Upper Right corner inside: Should be False...", obs_manager.get_state_validity([17.0216236115, 7.27234268188, 0.0])
  print "Bottom Right corner inside: Should be False...", obs_manager.get_state_validity([10.4382705688, 1.49695897102, 0.0])

  print "Testing middle square"
  print "Upper Left corner middle: Should be True...", obs_manager.get_state_validity([10.4908685684, 15.4024438858, 0.0])
  print "Bottom Left corner middle: Should be True...", obs_manager.get_state_validity([3.07843399048, 8.75644397736, 0.0])
  print "Upper Right corner middle: Should be True...", obs_manager.get_state_validity([17.6345787048, 7.33979320526, 0.0])
  print "Bottom Right corner middle: Should be True...", obs_manager.get_state_validity([9.97846317291, 0.0722773820162, 0.0])
  plt.show()