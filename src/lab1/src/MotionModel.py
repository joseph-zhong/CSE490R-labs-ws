#!/usr/bin/env python

import rospy
import numpy as np
import utils as Utils
import matplotlib.pyplot as pp
from std_msgs.msg import Float64
from threading import Lock

from pprint import pprint

# Delete this later
from nav_msgs.msg import Odometry

# Motion Model Hyperparameters.
ODOM_NOISE_MEAN = 0.0
ODOM_NOISE_STD = 1.5e-3
ODOM_NOISE_THETA_STD = 1e-3

KINEMATIC_NOISE_MEAN = 0.0
KINEMATIC_NOISE_POSITION_STD = 3e-2  # 3e-1 looks pretty good from RViz
KINEMATIC_NOISE_DELTA_STD = 1e-1  # 2e-1 seems pretty good for this based on looking at it in RViz
# for kinematic noises, position should be about 10,000 times greater than delta

# Car globals.
CAR_LEN = 0.33

class OdometryMotionModel:

  def __init__(self, particles, state_lock=None):
    self.last_pose = None # The last pose that was received
    self.particles = particles
    self.count = 0
    self.show_scatter_x = np.array([])
    self.show_scatter_y = np.array([])
    if state_lock is None:
      self.state_lock = Lock()
    else:
      self.state_lock = state_lock
    
  def motion_cb(self, msg):
    self.state_lock.acquire()
    pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, Utils.quaternion_to_angle(msg.pose.pose.orientation)])

    def rotation_matrix(theta):
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        return np.array([[cos_theta, sin_theta], [-sin_theta, cos_theta]])

    if isinstance(self.last_pose, np.ndarray):
      old_control = pose - self.last_pose
      #pprint(old_control)
      x_prime, y_prime, theta_prime = old_control
      theta = self.last_pose[2]
      delta_x, delta_y = rotation_matrix(theta).dot(np.array([x_prime, y_prime]))

      control = np.array([delta_x, delta_y, theta_prime])

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
    #pprint(control)
    noisy_control = control[0:2] + np.random.normal(loc=ODOM_NOISE_MEAN, scale=ODOM_NOISE_STD, size=self.particles[:, 0:2].shape)
    delta_x = np.cos(self.particles[:, 2]) * noisy_control[:, 0] + -np.sin(self.particles[:, 2]) * noisy_control[:, 1]
    delta_y = np.sin(self.particles[:, 2]) * noisy_control[:, 0] + np.cos(self.particles[:, 2]) * noisy_control[:, 1]
    noisy_control_theta = control[2] + np.random.normal(loc=ODOM_NOISE_MEAN, scale=ODOM_NOISE_THETA_STD, size=self.particles[:,0].shape)
    #pprint(noisy_control_theta)
    # pprint(delta_y)

    #pprint(self.particles)
    self.particles[:, 0] += delta_x
    self.particles[:, 1] += delta_y
    self.particles[:, 2] += noisy_control_theta
    self.particles[:, 2] %= (2 * np.pi)

    if self.count < 20:
      self.count += 1
      print self.count
      self.show_scatter_x = np.append(self.show_scatter_x, self.particles[:, 0])
      self.show_scatter_y = np.append(self.show_scatter_y, self.particles[:, 1])
    else:
      pprint((self.show_scatter_x))
      pprint(self.show_scatter_y.shape)
      pp.plot(self.show_scatter_x, self.show_scatter_y, 'ro')
      pp.show()

    #pprint(self.particles[:, 0])
    #pp.plot(self.particles[:, 0], self.particles[:, 1], 'ro', np.array([0]), np.array([0]), 'bs')
    #pp.show()
    #assert False

class KinematicMotionModel:

  def __init__(self, particles, state_lock=None):
    self.count = 0
    self.show_scatter_x = np.array([])
    self.show_scatter_y = np.array([])
    self.last_servo_cmd = None # The most recent servo command
    self.last_vesc_stamp = None # The time stamp from the previous vesc state msg
    self.particles = particles
    self.SPEED_TO_ERPM_OFFSET = 0.0 #float(rospy.get_param("/vesc/speed_to_erpm_offset")) # Offset conversion param from rpm to speed
    self.SPEED_TO_ERPM_GAIN = 4614 #float(rospy.get_param("/vesc/speed_to_erpm_gain"))   # Gain conversion param from rpm to speed
    self.STEERING_TO_SERVO_OFFSET = 0.5304 #float(rospy.get_param("/vesc/steering_angle_to_servo_offset")) # Offset conversion param from servo position to steering angle
    self.STEERING_TO_SERVO_GAIN = -1.2135 #float(rospy.get_param("/vesc/steering_angle_to_servo_gain")) # Gain conversion param from servo position to steering angle

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
    """
    Compute controls, with respect to dt
    :param msg:
    :return:
    """
    #print "motion_cb"
    self.state_lock.acquire()

    if self.last_servo_cmd is None:
      self.state_lock.release()
      return
    if self.last_vesc_stamp is None:
      self.last_vesc_stamp = msg.header.stamp
      # return


    # Convert raw msgs to controls
    # Note that control = (raw_msg_val - offset_param) / gain_param < This converts from raw motor speed to car velocity
    # Tim: Also use the same formula above to the steering angle
    # pprint(msg)
    curr_speed = (msg.state.speed - self.SPEED_TO_ERPM_OFFSET) / self.SPEED_TO_ERPM_GAIN
    curr_steering_angle = (self.last_servo_cmd - self.STEERING_TO_SERVO_OFFSET) / self.STEERING_TO_SERVO_GAIN
    dt = msg.header.stamp - self.last_vesc_stamp

    self.apply_motion_model(proposal_dist=self.particles, control=(curr_speed, curr_steering_angle, dt))
    self.last_vesc_stamp = msg.header.stamp
    self.state_lock.release()
    
  def apply_motion_model(self, proposal_dist, control):
    #print "applying motion model"
    # Update the proposal distribution by applying the control to each particle
    # YOUR CODE HERE
    pprint(control)
    # todo: review convert dt to value?
    v, delta, dt = control
    dt = dt.to_sec()

    position_noise = np.random.normal(loc=KINEMATIC_NOISE_MEAN, scale=KINEMATIC_NOISE_POSITION_STD, size=(len(self.particles), 1))
    delta_noise = np.random.normal(loc=KINEMATIC_NOISE_MEAN, scale=KINEMATIC_NOISE_DELTA_STD, size=(len(self.particles), 1))
    noisy_v = v + position_noise[:, 0]
    noisy_delta = delta + delta_noise[:, 0]

    theta = self.particles[:, 2]

    delta_theta = noisy_v / CAR_LEN * np.sin(noisy_delta) * dt
    delta_x = CAR_LEN / np.sin(noisy_delta) * (np.sin(theta + delta_theta) - np.sin(theta))
    delta_y = CAR_LEN / np.sin(noisy_delta) * (np.cos(theta) - np.cos(theta + delta_theta))

    self.particles[:, 0] += delta_x
    self.particles[:, 1] += delta_y
    self.particles[:, 2] += delta_theta
    self.particles[:, 2] %= (2 * np.pi)


    if self.count < 20:
      self.count += 1
      print self.count
      self.show_scatter_x = np.append(self.show_scatter_x, self.particles[:, 0])
      self.show_scatter_y = np.append(self.show_scatter_y, self.particles[:, 1])
    else:
      pprint((self.show_scatter_x))
      pprint(self.show_scatter_y.shape)
      pp.plot(self.show_scatter_x, self.show_scatter_y, 'ro')
      pp.show()

    #p.plot(self.particles[:, 0], self.particles[:, 1], 'ro', np.array([0]), np.array([0]), 'bs')
    #pp.show()
    #pprint(self.particles)

    # pprint(self.particles)