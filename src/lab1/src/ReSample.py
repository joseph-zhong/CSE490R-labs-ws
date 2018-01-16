import rospy
import numpy as np

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
    
    self.state_lock.release()
  
  def resample_low_variance(self):
    self.state_lock.acquire()
    # Implement low variance re-sampling
    # YOUR CODE HERE
    
    self.state_lock.release()

