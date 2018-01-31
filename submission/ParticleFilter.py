#!/usr/bin/env python

import rospy

import numpy as np
import matplotlib.pyplot as plt

import time

import utils
import utils as Utils
import tf.transformations
import tf
from pprint import pprint
from threading import Lock

from vesc_msgs.msg import VescStateStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.srv import GetMap
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseArray, PoseWithCovarianceStamped, PointStamped, Pose

from ReSample import ReSampler
from SensorModel import SensorModel
from MotionModel import OdometryMotionModel, KinematicMotionModel

STARTING_NOISE_MEAN = 0.0
STARTING_NOISE_STD = 1e-2

MAXIMUM_POINTS_TO_VISUALIZE = 500
 
class ParticleFilter(object):
  def __init__(self):
    self.MAX_PARTICLES = int(rospy.get_param("~max_particles")) # The maximum number of particles
    self.MAX_VIZ_PARTICLES = int(rospy.get_param("~max_viz_particles")) # The maximum number of particles to visualize

    self.particle_indices = np.arange(self.MAX_PARTICLES)
    self.particles = np.zeros((self.MAX_PARTICLES, 3))  # Numpy matrix of dimension MAX_PARTICLES x 3
    self.weights = np.ones(self.MAX_PARTICLES) / float(self.MAX_PARTICLES)  # Numpy matrix containig weight for each particle

    self.state_lock = Lock() # A lock used to prevent concurrency issues. You do not need to worry about this

    # Use the 'static_map' service (launched by MapServer.launch) to get the map
    # Will be used to initialize particles and SensorModel
    # Store map in variable called 'map_msg'
    # Tim: I think we use nav_msgs/GetMap
    # YOUR CODE HERE
    map_msg_srv = rospy.ServiceProxy("static_map", GetMap)
    try:
      map_msg = map_msg_srv().map
    except rospy.ServiceException as exc:
      raise rospy.ServiceException("Service did not process: request " + str(exc))

    # Globally initialize the particles
    self.initialize_global(map_msg)
   
    # Publish particle filter state
    self.pose_pub      = rospy.Publisher("/pf/viz/inferred_pose", PoseStamped, queue_size = 1) # Publishes the expected pose
    self.particle_pub  = rospy.Publisher("/pf/viz/particles", PoseArray, queue_size = 1) # Publishes a subsample of the particles
    self.pub_tf = tf.TransformBroadcaster() # Used to create a tf between the map and the laser for visualization
    self.pub_laser     = rospy.Publisher("/pf/viz/scan", LaserScan, queue_size = 1) # Publishes the most recent laser scan

    self.RESAMPLE_TYPE = rospy.get_param("~resample_type", "naiive") # Whether to use naiive or low variance sampling
    self.resampler = ReSampler(self.RESAMPLE_TYPE, self.particles, self.weights, self.state_lock)  # An object used for resampling

    self.sensor_model = SensorModel(map_msg, self.particles, self.weights, self.state_lock) # An object used for applying sensor model
    self.laser_sub = rospy.Subscriber(rospy.get_param("~scan_topic", "/scan"), LaserScan, self.sensor_model.lidar_cb, queue_size=1)

    self.MOTION_MODEL_TYPE = rospy.get_param("~motion_model", "kinematic") # Whether to use the odometry or kinematics based motion model
    if self.MOTION_MODEL_TYPE == "kinematic":
      self.motion_model = KinematicMotionModel(self.particles, self.state_lock) # An object used for applying kinematic motion model
      self.motion_sub = rospy.Subscriber(rospy.get_param("~motion_topic", "/vesc/sensors/core"), VescStateStamped, self.motion_model.motion_cb, queue_size=1)
      # Tim: ^ The topic above is where we get our velocity from
    elif self.MOTION_MODEL_TYPE == "odometry":
      self.motion_model = OdometryMotionModel(self.particles, self.state_lock)# An object used for applying odometry motion model
      self.motion_sub = rospy.Subscriber(rospy.get_param("~motion_topic", "/vesc/odom"), Odometry, self.motion_model.motion_cb, queue_size=1)
    else:
      raise Exception("Unrecognized motion model: "+ self.MOTION_MODEL_TYPE)

    # Use to initialize through rviz. Check clicked_pose_cb for more info
    self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.clicked_pose_cb, queue_size=1)

    # Subscribers to /pf/ta/viz/particles to compute RMS error.
    self.ta_particle_sub = rospy.Subscriber("/pf/ta/viz/particles", PoseArray, self.ta_particle_cb, queue_size=1)
    self.ta_particles = np.zeros_like(self.particles)
    self.rms = None

    # Fields for tracking resample-iteration timing and error over time.
    self.realtimes = []
    self.resample_time = []
    self.rmses = []

  # Initialize the particles to cover the map
  def initialize_global(self, map_msg):
    # Tim: Haltan sequence
    # YOUR CODE HERE
    pass
    
  # Publish a tf between the laser and the map
  # This is necessary in order to visualize the laser scan within the map
  def publish_tf(self, pose):
    x, y, theta = pose
    translation = (x, y, 0)
    rotation = tf.transformations.quaternion_from_euler(0, 0, theta)
    self.pub_tf.sendTransform(translation, rotation, rospy.Time.now(), "laser", "map")

  # Returns the expected pose given the current particles and weights
  def expected_pose(self):
    # fucking_shit = np.copy(self.motion_model.particles)
    # fucking_shit[:, 2] = np.arctan(
    #     np.average(np.sin(fucking_shit[:, 2]), axis=0) * self.weights /
    #     np.average(np.cos(fucking_shit[:, 2]), axis=0) * self.weights
    # return np.average(fucking_shit, axis=0)
    return np.average(self.motion_model.particles, axis=0)

  # Callback for '/initialpose' topic. RVIZ publishes a message to this topic when you specify an initial pose using its GUI
  # Reinitialize your particles and weights according to the received initial pose
  # Remember to apply a reasonable amount of Gaussian noise to each particle's pose
  def clicked_pose_cb(self, msg):
    self.state_lock.acquire()
    # print("Clicked pose message")
    #pprint(msg)
    start_x = msg.pose.pose.position.x
    start_y = msg.pose.pose.position.y
    start_theta = Utils.quaternion_to_angle(msg.pose.pose.orientation)
    starting_particles = np.random.normal(loc=STARTING_NOISE_MEAN, scale=STARTING_NOISE_STD, size=self.particles.shape)
    starting_particles[:, 0] += start_x
    starting_particles[:, 1] += start_y
    starting_particles[:, 2] += start_theta
    np.divide(self.sensor_model.weights, np.sum(self.sensor_model.weights), out=self.sensor_model.weights)
    self.motion_model.particles.fill(0)
    self.motion_model.particles += starting_particles
    # YOUR CODE HERE
    
    self.state_lock.release()
    
  # Visualize the current state of the filter
  # (1) Publishes a tf between the map and the laser. Necessary for visualizing the laser scan in the map
  # (2) Publishes the most recent laser measurement. Note that the frame_id of this message should be the child_frame_id of the tf from (1)
  # (3) Publishes a PoseStamped message indicating the expected pose of the car
  # (4) Publishes a subsample of the particles (use self.MAX_VIZ_PARTICLES). 
  #     Sample so that particles with higher weights are more likely to be sampled.
  def visualize(self):
    self.state_lock.acquire()
    assert self.weights.all() == self.sensor_model.weights.all()
    #self.publish_tf() # publishes the tf between the map and laser

    # You first get the expected pose of the car, using the right function
    # You then publish the transform from this pose to base_link, so that you know where the put the laser
    # You then put a PoseStamped where you think the car is
    # You then sample some of particles and display them as well

    expected_pose = self.expected_pose()  # Gets the expected pose of the robot
    self.publish_tf(expected_pose)        # Publishes a tf between map and laser, which is at expected pose
    #pprint(expected_pose)

    #Publishes the most recent laser measurement.
    #pprint(self.sensor_model.last_laser)
    self.sensor_model.last_laser.header.stamp = rospy.Time.now()
    #pprint(self.sensor_model.last_laser)
    self.pub_laser.publish(self.sensor_model.last_laser)

    # Publishes a PoseStamped message indicating the expected pose of the car
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = "map"
    pose_stamped.header.stamp = rospy.Time()

    pose_stamped.pose.position.x = expected_pose[0]
    pose_stamped.pose.position.y = expected_pose[1]

    quat = Utils.angle_to_quaternion(expected_pose[2])
    pose_stamped.pose.orientation.x = quat[0]
    pose_stamped.pose.orientation.y = quat[1]
    pose_stamped.pose.orientation.z = quat[2]
    pose_stamped.pose.orientation.w = quat[3]
    self.pose_pub.publish(pose_stamped)

    # Publishes a subsample of the particles in a PoseArray
    pa = PoseArray()
    pa.header.stamp = rospy.Time()
    pa.header.frame_id = "map"

    particles_to_viz = self.motion_model.particles[np.random.choice(self.motion_model.particles.shape[0],
                                                                    self.MAX_VIZ_PARTICLES, replace=False), :]
    #pprint(particles_to_viz)
    poses_array = []
    for i in range(0, len(particles_to_viz)):
      pose = Pose()
      pose.position.x = particles_to_viz[i, 0]
      pose.position.y = particles_to_viz[i, 1]
      quat = Utils.angle_to_quaternion(particles_to_viz[i, 2])
      pose.orientation.x = quat[0]
      pose.orientation.y = quat[1]
      pose.orientation.z = quat[2]
      pose.orientation.w = quat[3]
      poses_array.append(pose)

    pa.poses = poses_array
    #pprint(pa)
    self.particle_pub.publish(pa)

    # YOUR CODE HERE

    self.state_lock.release()

  def ta_particle_cb(self, msg):
    # print "ENTERED TA_PARTICLE_CB"
    for i, pose in enumerate(msg.poses):
      self.ta_particles[i] = pose.position.x, pose.position.y, utils.quaternion_to_angle(pose.orientation)

# Suggested main
  rospy.init_node("particle_filter", anonymous=True) # Initialize the node
if __name__ == '__main__':
  pf = ParticleFilter() # Create the particle filter
  
  while not rospy.is_shutdown(): # Keep going until we kill it
    # Callbacks are running in separate threads
    if pf.sensor_model.do_resample: # Check if the sensor model says it's time to resample
      pf.sensor_model.do_resample = False # Reset so that we don't keep resampling

      # Resample
      s_time = time.time()
      if pf.RESAMPLE_TYPE == "naiive":
        pf.resampler.resample_naiive()
      elif pf.RESAMPLE_TYPE == "low_variance":
        pf.resampler.resample_low_variance()
      else:
        raise Exception("Unrecognized resampling method: " + pf.RESAMPLE_TYPE)
      e_time = time.time()

      realtime = rospy.Time.now().secs
      pf.realtimes.append(realtime)
      print len(pf.realtimes)

      # Compute RMS.
      pf.rms = np.sqrt(np.sum(np.square(pf.particles - pf.ta_particles), axis=0) / len(pf.particles))
      pf.rmses.append(pf.rms)

      pf.visualize()  # Perform visualization

    # 4.6.1: Plot distribution for the re-sampled particles.
    # if len(pf.realtimes) == 15 or len(pf.realtimes) == 20:
    #   dst_fig = plt.figure(1)
    #   dst_fig.suptitle('Particle Resampling Distribution (n={}, t={})'.format(
    #     len(pf.resampler.particle_indices), len(pf.realtimes)), fontsize=17)
    #   plt.hist(pf.resampler.particle_indices, bins=len(pf.resampler.particle_indices))
    #   plt.xlabel('Particle Indices')
    #   plt.ylabel('Number of particles')
    #   plt.show()

    # 4.6.3: Plot RMS error over time for the re-sampled particles.
    # if len(pf.realtimes) == MAXIMUM_POINTS_TO_VISUALIZE:
    #   assert len(pf.realtimes) == len(pf.rmses), "Unexpected lengths to plot: '{}' vs '{}'".format(len(pf.realtimes), len(pf.rmses))
    #   rms_fig = plt.figure(2)
    #   rms_fig.suptitle('Particle Resampling RMS Error over time (n={})'.format(len(pf.resampler.particle_indices)), fontsize=17)
    #
    #
    #   pf.rmses = np.array(pf.rmses)
    #
    #   plt.plot(pf.realtimes, pf.rmses[:, 0])
    #   plt.plot(pf.realtimes, pf.rmses[:, 1])
    #   plt.plot(pf.realtimes, pf.rmses[:, 2])
    #
    #   plt.legend(('Pose: x', 'Pose: y', 'Pose: theta'))
    #   plt.xlabel('Rospy Time (s)')
    #   plt.ylabel('RMS Error')
    #   plt.show()
    # 4.6.3: Pickle RMS to plot later.
    # if len(pf.realtimes) == MAXIMUM_POINTS_TO_VISUALIZE:
    #   assert len(pf.realtimes) == len(pf.rmses), "Unexpected lengths to plot: '{}' vs '{}'".format(len(pf.realtimes), len(pf.rmses))
    #   pf.rmses = np.array(pf.rmses)
    #   np.save("rmses_{}".format(len(pf.particle_indices)), pf.rmses)
    #   np.save("realtimes", pf.realtimes)
    #   print 'fuck'
    #   assert False