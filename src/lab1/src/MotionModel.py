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

# Motion Model Hyperparameters.
ODOM_NOISE_MEAN = 0.0
ODOM_NOISE_STD = 1e-3

# Car globals.
CAR_LEN = 0.33

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
    pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, Utils.quaternion_to_angle(msg.pose.pose.orientation)])

    if isinstance(self.last_pose, np.ndarray):
      old_control = pose - self.last_pose
      delta_x = old_control[0] * math.cos(self.last_pose[2]) + old_control[1] * math.sin(self.last_pose[2])
      delta_y = -old_control[0] * math.sin(self.last_pose[2]) + old_control[1] * math.cos(self.last_pose[2])
      control = np.array([delta_x, delta_y, old_control[2]])

      # TODO: This just needs an extension.
      # rot = np.array([[np.cos(self.last_pose[2]), np.sin(self.last_pose[2])], [-np.sin(self.last_pose[2]), np.cos(self.last_pose[2])]])
      # test_control = np.matmul(rot, old_control[:2])
      #
      # pprint(control)
      # pprint(test_control)

      #assert np.all(control == test_control)

      # pprint(control)
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

    # TODO josephz: This needs the decoupling trick thingy
    # rot = np.array([[np.cos(self.particles[:][2]), np.sin(self.particles[:][2])],
    #            -np.sin(self.particles[:][2]), np.cos(self.particles[:][2])])
    # np.matmul(rot, self.particles[:, :2])

    noisy_control = control + np.random.normal(loc=ODOM_NOISE_MEAN, scale=ODOM_NOISE_STD, size=self.particles.shape)

    # TODO josephz: This can be faster as well... by doing a vector add.
    self.particles[:, 0] += np.cos(self.particles[:, 2]) * noisy_control[:, 0] + np.sin(self.particles[:, 2]) * noisy_control[:, 1]
    self.particles[:, 1] += -np.sin(self.particles[:, 2]) * noisy_control[:, 0] + np.cos(self.particles[:, 2]) * noisy_control[:, 1]
    self.particles[:, 2] += noisy_control[:, 2]
    pprint(self.particles)
    
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
      # return


    # Convert raw msgs to controls
    # Note that control = (raw_msg_val - offset_param) / gain_param < This converts from raw motor speed to car velocity
    # Tim: Also use the same formula above to the steering angle
    pprint(msg)
    curr_speed = (msg.state.speed - self.SPEED_TO_ERPM_OFFSET) / self.SPEED_TO_ERPM_GAIN
    curr_steering_angle = (self.last_servo_cmd - self.STEERING_TO_SERVO_OFFSET) / self.STEERING_TO_SERVO_GAIN
    # TODO josephz: what the fuck does this guy do?
    # dt = msg.header.stamp - self.last_vesc_stamp

    self.apply_motion_model(proposal_dist=self.particles, control=(curr_speed, curr_steering_angle))
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

    print a
    print b
    self.particles[:, 0] += a + b

    self.particles[:, 1] += -np.sin(self.particles[:, 2]) * noisy_control[:, 0] \
                            + np.cos(self.particles[:, 2]) * noisy_control[:, 1]
    self.particles[:, 2] += noisy_control[:, 2]
    pprint(self.particles)

