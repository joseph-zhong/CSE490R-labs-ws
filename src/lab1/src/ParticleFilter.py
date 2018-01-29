#!/usr/bin/env python

import rospy 
import numpy as np
import time
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
 
class ParticleFilter():

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
    self.resampler = ReSampler(self.particles, self.weights, self.state_lock)  # An object used for resampling

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
      print "Unrecognized motion model: "+ self.MOTION_MODEL_TYPE
      assert(False)
    
    # Use to initialize through rviz. Check clicked_pose_cb for more info
    self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.clicked_pose_cb, queue_size=1)

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
    return np.average(self.motion_model.particles, weights=self.weights, axis=0)
    # TODO: Tim: Get rid of the weights

  # Callback for '/initialpose' topic. RVIZ publishes a message to this topic when you specify an initial pose using its GUI
  # Reinitialize your particles and weights according to the received initial pose
  # Remember to apply a reasonable amount of Gaussian noise to each particle's pose
  def clicked_pose_cb(self, msg):
    self.state_lock.acquire()
    print("Clicked pose message")
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
    pprint(self.motion_model.particles)
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
    #pprint(self.sensor_model.last_laser.header)
    self.pub_laser.publish(self.sensor_model.last_laser)

    # Publishes a PoseStamped message indicating the expected pose of the car
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = "map"
    pose_stamped.header.stamp = rospy.Time.now()

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

# Suggested main
if __name__ == '__main__':
  rospy.init_node("particle_filter", anonymous=True) # Initialize the node
  pf = ParticleFilter() # Create the particle filter
  
  while not rospy.is_shutdown(): # Keep going until we kill it
    # Callbacks are running in separate threads
    if pf.sensor_model.do_resample: # Check if the sensor model says it's time to resample
      pf.sensor_model.do_resample = False # Reset so that we don't keep resampling

      # Resample
      if pf.RESAMPLE_TYPE == "naiive":
        #pf.resampler.resample_naiive()
        pass
      elif pf.RESAMPLE_TYPE == "low_variance":
        #pf.resampler.resample_low_variance()
        pass
      else:
        print "Unrecognized resampling method: "+ pf.RESAMPLE_TYPE

    pf.visualize()  # Perform visualization



