#!/usr/bin/env python

import rospy
import numpy as np
import utils as Utils
from std_msgs.msg import Float64
from threading import Lock

class OdometryMotionModel:

  def __init__(self, particles, state_lock=None):
    self.last_pose = None # The last pose that was received
    self.particles = particles
    if state_lock is None:
      self.state_lock = Lock()
    else:
      self.state_lock = state_lock
    
  def motion_cb(self, msg):
    self.state_lock.acquire()
      
    if isinstance(self.last_pose, np.ndarray):
      
      # Compute the control from the msg and last_pose
      # YOUR CODE HERE
      
      self.apply_motion_model(self.particles, control)

    self.last_pose = pose
    self.state_lock.release()
    
  def apply_motion_model(self, proposal_dist, control):
    # Update the proposal distribution by applying the control to each particle
    # YOUR CODE HERE
    pass
    
class KinematicMotionModel:

  def __init__(self, particles, state_lock=None):
    self.last_servo_cmd = None # The most recent servo command
    self.last_vesc_stamp = None # The time stamp from the previous vesc state msg
    self.particles = particles
    self.SPEED_TO_ERPM_OFFSET = float(rospy.get_param("/vesc/speed_to_erpm_offset")) # Offset conversion param from rpm to speed
    self.SPEED_TO_ERPM_GAIN   = float(rospy.get_param("/vesc/speed_to_erpm_gain"))   # Gain conversion param from rpm to speed
    self.STEERING_TO_SERVO_OFFSET = float(rospy.get_param("/vesc/steering_angle_to_servo_offset")) # Offset conversion param from servo position to steering angle
    self.STEERING_TO_SERVO_GAIN   = float(rospy.get_param("/vesc/steering_angle_to_servo_gain")) # Gain conversion param from servo position to steering angle

    if state_lock is None:
      self.state_lock = Lock()
    else:
      self.state_lock = state_lock
      
    # This subscriber just caches the most recent servo position command
    self.servo_pos_sub  = rospy.Subscriber(rospy.get_param("~servo_pos_topic", "/vesc/sensors/servo_position_command"), Float64,
                                       self.servo_cb, queue_size=1)
                                       

  def servo_cb(self, msg):
    self.last_servo_cmd = msg.data # Just update servo command

  def motion_cb(self, msg):
    """
    Compute controls, with respect to dt
    :param msg:
    :return:
    """
    self.state_lock.acquire()
    
    if self.last_servo_cmd is None:
      return

    if self.last_vesc_stamp is None:
      self.last_vesc_stamp = msg.header.stamp

    # Convert raw msgs to controls
    # Note that control = (raw_msg_val - offset_param) / gain_param
    # YOUR CODE HERE
    
    self.apply_motion_model(proposal_dist=self.particles, control=[curr_speed, curr_steering_angle, dt])
    self.last_vesc_stamp = msg.header.stamp
    self.state_lock.release()
    
  def apply_motion_model(self, proposal_dist, control):
    # Update the proposal distribution by applying the control to each particle
    # YOUR CODE HERE

    v, delta = control
    beta = np.arctan(np.tan(delta/ 2))

    # Kinetic to Odom conversion to remain consistent with particle update.
    odom_control = np.array([
      v * np.cos(self.particles[:, 2]), # t - 1
      v * np.sin(self.particles[:, 2]),
      v / CAR_LEN * np.sin(2 * beta)       # time t ->
    ])
    print control
    print odom_control

    # Compute particle updates.
    # TODO josephz: This can be faster as well... by doing a vector add.
    noiasdf = np.random.normal(loc=ODOM_NOISE_MEAN, scale=ODOM_NOISE_STD, size=self.particles.shape)
    noisy_control = odom_control + noiasdf
    a = np.cos(self.particles[:, 2]) * noisy_control[:, 0]
    b = np.sin(self.particles[:, 2]) * noisy_control[:, 1]

    print a, b
    print a.dtype, b.dtype
    self.particles[:, 0] += a + b

    self.particles[:, 1] += -np.sin(self.particles[:, 2]) * noisy_control[:, 0] \
                            + np.cos(self.particles[:, 2]) * noisy_control[:, 1]
    self.particles[:, 2] += noisy_control[:, 2]
    pprint(self.particles)

>>>>>>> Stashed changes
