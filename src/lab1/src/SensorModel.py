#!/usr/bin/env python

import numpy as np
import rospy
import range_libc
from scipy.stats import norm
import time
from threading import Lock

THETA_DISCRETIZATION = 112 # Discretization of scanning angle
INV_SQUASH_FACTOR = 2.2    # Factor for helping the weight distribution to be less peaked

Z_SHORT = 1  # Weight for short reading
Z_MAX = 1    # Weight for max reading
Z_RAND = 1   # Weight for random reading
SIGMA_HIT = 1# Noise value for hit reading
Z_HIT = 1    # Weight for hit reading

class SensorModel:
	
  def __init__(self, map_msg, particles, weights, state_lock=None):
    if state_lock is None:
      self.state_lock = Lock()
    else:
      self.state_lock = state_lock
  
    self.particles = particles
    self.weights = weights
    
    self.LASER_RAY_STEP = int(rospy.get_param("~laser_ray_step")) # Step for downsampling laser scans
    self.MAX_RANGE_METERS = float(rospy.get_param("~max_range_meters")) # The max range of the laser
    
    oMap = range_libc.PyOMap(map_msg) # A version of the map that range_libc can understand
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
        print msg
        self.downsampled_angles = np.arange(msg.angle_min, msg.angle_max, self.LASER_RAY_STEP, dtype=np.float32)

    downsampled_ranges = np.array(msg.ranges[::self.LASER_RAY_STEP], dtype=np.float32)
    downsampled_angles = self.downsampled_angles
    obs = (downsampled_ranges, downsampled_angles)

    self.apply_sensor_model(self.particles, obs, self.weights)
    self.weights /= np.sum(self.weights)
    
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

    # Populate sensor model table as specified
    # Note that the row corresponds to the observed measurement and the column corresponds to the expected measurement
    # YOUR CODE HERE

    # sensor_model_table[z_obs][z_exp] -> p(z | x)
    # Get z_exp using range_libc on a pose, z_obs?????

    # Populate the matrix
    def interpolated_pdf(observed, expected):
        # Sample from normal pdf
        p_hit = norm(observed, expected, SIGMA_HIT)

        p_short = 0
        # Short is nonzero if observed is within 0 and expected
        if 0 <= observed and observed <= expected:
            p_short = lambda_short * np.exp(-lambda_short * observed)

        p_rand = 0
        # Rand is nonzero if observed is within 0 and z_max
        if 0 <= observed and observed <= max_range_px:
            p_rand = 1.0 / max_range_px
                    
        # p_max is 1 only if observed = z_max, 0 otherwise
        p_max = 1 if observed == max_range_px else 0

        return Z_HIT * p_hit + Z_SHORT * p_short + Z_RAND * p_rand + Z_MAX * p_max


    # Calculate each entry for the table and normalize by columns.
    sensor_model_table = np.fromfunction(interpolated_pdf, (table_width, table_width), dtype=np.float32)
    column_sums = sensor_model_table.sum(axis=0)
    sensor_model_table /= column_sums

    return sensor_model_table

  def apply_sensor_model(self, proposal_dist, obs, weights):
        
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

if __name__ == '__main__':
  pass
