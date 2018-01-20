#!/usr/bin/env python

import rospy
import numpy as np
import utils as Utils
import math
from std_msgs.msg import Float64
from threading import Lock

from pprint import pprint

# Delete this later
from nav_msgs.msg import Odometry



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
    pprint(msg)
    pose = np.array([msg.pose.position.x, msg.pose.position.y, Utils.quaternion_to_angle(msg.pose.Quaternion)])

    if isinstance(self.last_pose, np.ndarray):
      old_control = pose - self.last_pose
      delta_x = old_control[0] * math.cos(self.last_pose[2]) + old_control[1] * math.sin(self.last_pose[2])
      delta_y = -old_control[0] * math.sin(self.last_pose[2]) + old_control[1] * math.cos(self.last_pose[2])
      control = np.array([delta_x, delta_y, old_control[2]])
      pprint(control)
      # Compute the control from the msg and last_pose
      # YOUR CODE HERE
      # Compute control here
      
      self.apply_motion_model(self.particles, control)

    self.last_pose = pose
    self.state_lock.release()
    
  def apply_motion_model(self, proposal_dist, control):
    # Update the proposal distribution by applying the control to each particle
    # Tim: Add noise here
    # YOUR CODE HERE
    pass
    
class KinematicMotionModel:

  def __init__(self, particles, state_lock=None):
    self.last_servo_cmd = None # The most recent servo command
    self.last_vesc_stamp = None # The time stamp from the previous vesc state msg
    self.particles = particles
    self.SPEED_TO_ERPM_OFFSET = float(rospy.get_param("/vesc/speed_to_erpm_offset")) # Offset conversion param from rpm to speed
    self.SPEED_TO_ERPM_GAIN = float(rospy.get_param("/vesc/speed_to_erpm_gain"))   # Gain conversion param from rpm to speed
    self.STEERING_TO_SERVO_OFFSET = float(rospy.get_param("/vesc/steering_angle_to_servo_offset")) # Offset conversion param from servo position to steering angle
    self.STEERING_TO_SERVO_GAIN = float(rospy.get_param("/vesc/steering_angle_to_servo_gain")) # Gain conversion param from servo position to steering angle
    
    if state_lock is None:
      self.state_lock = Lock()
    else:
      self.state_lock = state_lock
      
    # This subscriber just caches the most recent servo position command
    self.servo_pos_sub = rospy.Subscriber(rospy.get_param("~servo_pos_topic", "/vesc/sensors/servo_position_command"), Float64,
                                       self.servo_cb, queue_size=1) # Tim: This tells you the position of the servo controlling the steering angle
                                       

  def servo_cb(self, msg):
    self.last_servo_cmd = msg.data # Just update servo command

  def motion_cb(self, msg):
    self.state_lock.acquire()
    
    if self.last_servo_cmd is None:
      return

    if self.last_vesc_stamp is None:
      self.last_vesc_stamp = msg.header.stamp
      return


    # Convert raw msgs to controls
    # Note that control = (raw_msg_val - offset_param) / gain_param < This converts from raw motor speed to car velocity
    # Tim: Also use the same formula above to the steering angle

    curr_speed = (msg.speed - self.SPEED_TO_ERPM_OFFSET) / self.SPEED_TO_ERPM_GAIN
    curr_steering_angle = (self.last_servo_cmd - self.STEERING_TO_SERVO_OFFSET) / self.STEERING_TO_SERVO_GAIN
    dt = msg.header.stamp - self.last_vesc_stamp

    self.apply_motion_model(proposal_dist=self.particles, control=[curr_speed, curr_steering_angle, dt])
    self.last_vesc_stamp = msg.header.stamp
    self.state_lock.release()
    
  def apply_motion_model(self, proposal_dist, control):
    # Update the proposal distribution by applying the control to each particle
    # YOUR CODE HERE
    pass
    
if __name__ == '__main__':
  pass
    
