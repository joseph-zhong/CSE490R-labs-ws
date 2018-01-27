import time
import rospy
import random
import numpy as np
from threading import Lock

class ReSampler:
  def __init__(self, particles, weights, state_lock=None):
    self.particles = particles
    self.weights = weights
    self.particle_indices = None  
    self.step_array = None
    
    if state_lock is None:
      self.state_lock = Lock()
    else:
      self.state_lock = state_lock
  
  def resample_naiive(self):
    self.state_lock.acquire()
    # Use np.random.choice to re-sample 
    # YOUR CODE HERE
    # Set the new particles via the indices.
    s_time = time.time()
    indices = np.random.choice(len(self.particles), size=len(self.particles), p=self.weights)
    e_time = time.time()
    np.take(self.particles, indices,  axis=0, out=self.particles)

    var = np.var(self.particles, axis=0)
    compute_time = e_time - s_time
    print "naiive re-sampler variance: '{}' computed in '{}' seconds".format(var, compute_time)
    with open('resample_naiive_variance.csv', 'a') as fin:
      fin.write('{};{}'.format(var, compute_time))

    self.state_lock.release()
  
  def resample_low_variance(self):
    self.state_lock.acquire()
    # Implement low variance re-sampling
    # YOUR CODE HERE

    # Initial attempt from the text.
    #
    # Choose a random weight [0, 1/M].
    # Iterate through each weight, add the particle if its weight is greater than the
    # incremented random choice. Repeat m total times.
    # M = len(self.particles)
    # i = 0
    # r = random.random() / M
    # cumsum = np.cumsum(self.weights)
    # resampled_particles = np.zeros(M, 3)
    #
    # for m, w in enumerate(self.weights):
    #   u = r + float(m) / M
    #   while cumsum[i] < u:
    #     i += 1
    #     assert i < M
    #   resampled_particles[m] = self.particles[i]
    # self.particles = resampled_particles

    # TODO josephz: Check that this version is faster.
    # Inspiration: moving indices should be slightly faster.
    # See https://github.com/rlabbe/filterpy/blob/master/filterpy/monte_carlo/resampling.py
    s_time = time.time()
    M = len(self.particles)
    r = random.random()
    positions = (r + np.arange(M)) / M
    indices = np.zeros(M, 'i')
    cumsum = np.cumsum(self.weights)
    i = j = 0

    # Iterate m times. If the increment is within the cumulative weight, add the index
    # to the particle. Otherwise, increment the cumulative weight.
    while i < M:
      if positions[i] < cumsum[j]:
        indices[i] = j
        i += 1
      else:
        j += 1
    e_time = time.time()
    # Set the new particles via the indices.
    np.take(self.particles, indices, axis=0, out=self.particles)

    var = np.var(self.particles, axis=0)
    compute_time = e_time - s_time
    print "low-var re-sampler variance: '{}', computed in '{}' seconds".format(var, compute_time)
    with open('resample_naiive_variance.csv', 'a') as fin:
      fin.write('{};{}'.format(var, compute_time))

    self.state_lock.release()
