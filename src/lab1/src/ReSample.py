import time
import rospy
import random

import numpy as np
import matplotlib.pyplot as plt
from threading import Lock

MAX_SAMPLES = 100

class ReSampler:
  def __init__(self, resample_type, particles, weights, state_lock=None):
    self.resample_type = resample_type
    self.particles = particles
    self.weights = weights
    self.particle_indices = None
    self.step_array = None

    if state_lock is None:
      self.state_lock = Lock()
    else:
      self.state_lock = state_lock
  
  def resample_naiive(self):
    print "Resampling naiively"

    self.state_lock.acquire()
    #print "resampling"
    # Use np.random.choice to re-sample 
    # YOUR CODE HERE
    # Set the new particles via the indices.
    self.particle_indices = np.random.choice(len(self.particles), size=len(self.particles), p=self.weights)
    np.take(self.particles, self.particle_indices,  axis=0, out=self.particles)

    self.state_lock.release()
  
  def resample_low_variance(self):
    print "Resampling with low-variance"

    self.state_lock.acquire()
    # Implement low variance re-sampling
    # YOUR CODE HERE

    # Initial attempt from the text.
    #
    # Choose a random weight [0, 1/M].
    # Iterate through each weight, add the particle if its weight is greater than the
    # incremented random choice. Repeat m total times.
    # s_time = time.time()
    # M = len(self.particles)
    # i = 0
    # r = random.random() / M
    # cumsum = np.cumsum(self.weights)
    # resampled_particles = np.zeros(shape=(M, 3))
    #
    # for m, w in enumerate(self.weights):
    #   u = r + float(m) / M
    #   while cumsum[i] < u:
    #     i += 1
    #     assert i < M
    #   resampled_particles[m] = self.particles[i]
    # e_time = time.time()
    # self.particles = resampled_particles

    # TODO josephz: Check that this version is faster.
    # Inspiration: moving indices should be slightly faster.
    # See https://github.com/rlabbe/filterpy/blob/master/filterpy/monte_carlo/resampling.py
    s_time = time.time()
    M = len(self.particles)
    r = random.random()
    positions = (r + np.arange(M)) / M
    self.particle_indices = np.zeros(M, 'i')
    cumsum = np.cumsum(self.weights)
    i = j = 0

    # Iterate m times. If the increment is within the cumulative weight, add the index
    # to the particle. Otherwise, increment the cumulative weight.
    while i < M:
      if positions[i] < cumsum[j]:
        self.particle_indices[i] = j
        i += 1
      else:
        j += 1
    # Set the new particles via the indices.
    np.take(self.particles, self.particle_indices, axis=0, out=self.particles)

    self.state_lock.release()
