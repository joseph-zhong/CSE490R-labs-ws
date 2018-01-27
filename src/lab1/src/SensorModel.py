#!/usr/bin/env python

import numpy as np
import rospy
import range_libc
from scipy.stats import norm
import time
from threading import Lock
from sensor_msgs.msg import LaserScan


THETA_DISCRETIZATION = 112 # Discretization of scanning angle
INV_SQUASH_FACTOR = 0.2    # Factor for helping the weight distribution to be less peaked

Z_SHORT = 0.0  # Weight for short reading
Z_MAX = 0.1    # Weight for max reading
Z_RAND = 0.1   # Weight for random reading
LAMBDA_SHORT = 0.5 # Parameter for short distribution
SIGMA_HIT = 3.0 # Noise value for hit reading
Z_HIT = 0.8    # Weight for hit reading

class SensorModel:
  def __init__(self, map_msg, particles, weights, state_lock=None):
    if state_lock is None:
      self.state_lock = Lock()
    else:
      self.state_lock = state_lock
  
    self.particles = particles
    self.weights = weights

    self.last_laser = LaserScan()

    self.LASER_RAY_STEP = int(rospy.get_param("~laser_ray_step")) # Step for downsampling laser scans
    self.MAX_RANGE_METERS = float(rospy.get_param("~max_range_meters")) # The max range of the laser
    
    oMap = range_libc.PyOMap(map_msg) # A version of the map that range_libc can understand
    # Tim: Compute expected range measurements and weights
    max_range_px = int(self.MAX_RANGE_METERS / map_msg.info.resolution) # The max range in pixels of the laser
    self.range_method = range_libc.PyCDDTCast(oMap, max_range_px, THETA_DISCRETIZATION) # The range method that will be used for ray casting
    self.range_method.set_sensor_model(self.precompute_sensor_model(max_range_px)) # Load the sensor model expressed as a table
    self.queries = None
    self.ranges = None
    self.laser_angles = None # The angles of each ray
    self.downsampled_angles = None # The angles of the downsampled rays 
    self.do_resample = False # Set so that outside code can know that it's time to resample
    
  def lidar_cb(self, msg):
    self.state_lock.acquire()

    # Compute the observation
    # obs is a a two element tuple
    # obs[0] is the downsampled ranges
    # obs[1] is the downsampled angles
    # Each element of obs must be a numpy array of type np.float32 (this is a requirement for GPU processing)
    # Use self.LASER_RAY_STEP as the downsampling step
    # Keep efficiency in mind, including by caching certain things that won't change across future iterations of this callback
  
    # YOUR CODE HERE
    # Tam - Downsampled angles shouldn't change.
    if self.downsampled_angles is None:
      # print msg
      self.downsampled_angles = np.arange(msg.angle_min, msg.angle_max, self.LASER_RAY_STEP, dtype=np.float32)

    downsampled_ranges = np.array(msg.ranges[::self.LASER_RAY_STEP], dtype=np.float32)
    #print "################## DOWNSAMPLED"
    #print len(downsampled_ranges)
    #print self.weights
    downsampled_angles = self.downsampled_angles
    obs = (downsampled_ranges, downsampled_angles)

    #print obs

    self.apply_sensor_model(self.particles, obs, self.weights)
    np.divide(self.weights, np.sum(self.weights), out=self.weights)
    #print 'weights'
    #print self.weights
    #print 'weights'

    self.last_laser = msg
    self.do_resample = True
    
    self.state_lock.release()
    
  def precompute_sensor_model(self, max_range_px):
    """
    Returns a table with columns as expectation while rows as observation measurement
    :param max_range_px:
    :return:
    """

    table_width = int(max_range_px) + 1
    sensor_model_table = np.zeros((table_width,table_width))


    # TODO: All four models here; calculate interpolated E[z_t | x_t]
    # Tam - Calculate z_t^k* using x_t (from downsampled angles???)

    # Populate sensor model table as specified
    # Note that the row corresponds to the observed measurement and the column corresponds to the expected measurement
    # YOUR CODE HERE

    # sensor_model_table[z_obs][z_exp] -> p(z | x)
    # Get z_exp using range_libc on a pose, z_obs?????

    # Populate the matrix
    def interpolated_pdf(observed, expected):
      # Sample from normal pdf
      p_hit = norm.pdf(observed, expected, SIGMA_HIT)
      condition = observed <= expected
      p_short = condition * LAMBDA_SHORT * np.exp(-LAMBDA_SHORT * observed)

      # Uniformly distributed
      p_rand = 1.0 / max_range_px
      # p_max is 1 only if observed = z_max, 0 otherwise
      p_max = observed == max_range_px

      #print p_hit
      #print p_short
      #print p_rand
      #print p_max

      return Z_HIT * p_hit + Z_SHORT * p_short + Z_MAX * p_max + Z_RAND * p_rand


    # Calculate each entry for the table and normalize by columns.
    sensor_model_table = np.fromfunction(interpolated_pdf, (table_width, table_width), dtype=np.float32)
    column_sums = sensor_model_table.sum(axis=0)
    sensor_model_table /= column_sums
    print(sensor_model_table[:10])

    return sensor_model_table

  def apply_sensor_model(self, proposal_dist, obs, weights):
    print "start________________________________________________________________________"
    #print proposal_dist
    #print obs
    #print weights
    #print self.ranges

    obs_ranges = obs[0]
    obs_angles = obs[1]
    num_rays = obs_angles.shape[0]
    
    # Only allocate buffers once to avoid slowness
    if not isinstance(self.queries, np.ndarray):
      self.queries = np.zeros((proposal_dist.shape[0],3), dtype=np.float32)
      self.ranges = np.zeros(num_rays*proposal_dist.shape[0], dtype=np.float32)

    self.queries[:,:] = proposal_dist[:,:]

    self.range_method.calc_range_repeat_angles(self.queries, obs_angles, self.ranges)

    # Evaluate the sensor model on the GPU
    self.range_method.eval_sensor_model(obs_ranges, self.ranges, weights, num_rays, proposal_dist.shape[0])

    np.power(weights, INV_SQUASH_FACTOR, weights)
    #print "################## WEIGHTS ##################"
    assert np.may_share_memory(weights,self.weights)

if __name__ == '__main__':
  pass
