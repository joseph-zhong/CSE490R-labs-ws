
from PIL import Image
from matplotlib.pyplot import cm
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import sys

from nav_msgs.srv import GetMap
import range_libc
import rosbag
import rospy
from sensor_msgs.msg import LaserScan

from SensorModelHeatMap import SensorModelHeatMap

THETA_DISCRETIZATION = 112
CONVERSION = 2 * np.pi / THETA_DISCRETIZATION

RESOLUTION = 0.0
OFFSET_X = 0.0
OFFSET_Y = 0.0
OFFSET_THETA = 0.0

DOWNSAMPLE = 1  # Rate to downsample plot for debugging
COUNT = 0  # Number of points plotted

def main():

  if len(sys.argv) < 3:
    print "Usage: python HeatMap.py <laser_scanX.bag> <output_name>"
    sys.exit()

  print "Hello!"

  # Output figure
  output_name = sys.argv[2]

  # Get the map
  laser_bag_path = sys.argv[1]
  map_msg_srv = rospy.ServiceProxy("static_map", GetMap)
  try:
    map_msg = map_msg_srv().map
  except rospy.ServiceException as exc:
    raise rospy.ServiceException("Service did not process: request " + str(exc))


  # Process Bag
  laser_bag = rosbag.Bag(laser_bag_path)
  topic, msg, t = next(laser_bag.read_messages())

  print map_msg.info
  RESOLUTION = map_msg.info.resolution
  OFFSET_X = map_msg.info.origin.position.x
  OFFSET_Y = map_msg.info.origin.position.y


  # Create two channel image containing map values and
  # likelihoods
  map_shape = (map_msg.info.width, map_msg.info.height)

  # For plotting
  map_data = np.array(map_msg.data).reshape(map_shape)
  map_data = map_data >= 0
  empty_spaces = np.where(map_data)
  image_plot = np.stack((np.zeros(map_shape),) * 3)
  image_plot[0] = map_data

  # Reserve a copy of non-expanded space for plotting
  # Convert from map to world
  map_to_world = lambda p: [p[0] * RESOLUTION + OFFSET_X, p[1] * RESOLUTION + OFFSET_Y]
  expanded_spaces = list(map(map_to_world, expanded_spaces))
  expanded_spaces = expanded_spaces[::100]

  # Create particles, THETA_DISCRETIZATION thetas per particle
  print "PARTICLES"
  particles = np.empty((0, 3), dtype=np.float32)
  
  # Angles will never change, pre-calculate them here.
  angles = np.linspace(0, 2 * np.pi, THETA_DISCRETIZATION)
  
  # Reshape for convenience
  angles = angles.reshape(len(angles), 1)


  # For each [x, y], create a list of [[x, y, theta_1] ... [x, y, theta_n]]
  progress = "\rIteration {}: Point: {}"
  for i, position in enumerate(expanded_spaces):
    position_copies = np.array([position] * THETA_DISCRETIZATION)
    next_particles = np.concatenate((position_copies, angles), axis=1)

    # Print progress
    print progress.format(i, position),
    sys.stdout.flush()

    particles = np.append(particles, next_particles, axis=0)

  print "FIRST CHUNK OF PARTICLES-------------"
  print particles[0:THETA_DISCRETIZATION]
  print

  # Init. the model and process the scan
  weights = np.ones(len(particles)) / float(len(particles))
  sm = SensorModelHeatMap(map_msg, particles, weights)
  sm.lidar_cb(msg)

  # Partition w.r.t theta
  calculated_weights = np.split(sm.weights, len(sm.weights) // THETA_DISCRETIZATION)


  #########################
  ### SENSOR MODEL OBS
  downsampled_angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment * 18, dtype=np.float32)
  downsampled_ranges = np.array(msg.ranges[::18], dtype=np.float32)
  isNan = np.isnan(downsampled_ranges)
  downsampled_ranges[isNan] = msg.range_max
  obs = (downsampled_ranges, downsampled_angles)

  plot_results(empty_spaces[1][::DOWNSAMPLE], empty_spaces[0][::DOWNSAMPLE], resulting_weights, map_data)

  # Create particles from an expanded set of the empty space
  # THETA_DISCRETIZATION thetas per particle
 
  # Angles will never change, pre-calculate them here.
  angles = np.linspace(0, 2 * np.pi, THETA_DISCRETIZATION)

  # Instantiate a Sensor Model
  sm = SensorModelHeatMap(map_msg, [], [])
  
  # For each [x, y], create a list of [[x, y, theta_1] ... [x, y, theta_n]]
  progress = "\rIteration {}: Point: {}"
  for i, position in enumerate(expanded_spaces):
    curr_particles = []
    curr_weights = np.zeros((len(angles),))
    for theta in angles:
      next_particle = np.append(position, [theta], axis=0)
      curr_particles.append(next_particle)

    # Calculate the weight for this particle
    curr_particles = np.array(curr_particles, dtype=np.float32)
    weights = np.ones(len(curr_particles)) / float(len(curr_particles))
    sm.apply_sensor_model(curr_particles, obs, curr_weights)
    position_weight = max(curr_weights)

    # Update the weight for this particle in the image (2nd channel)
    # using map coordinates.
    column, row = empty_spaces[1][i], empty_spaces[0][i]
    image_plot[1][row][column] = position_weight

    # Print progress
    print progress.format(i, position),
    sys.stdout.flush()

  print

  plot_results(empty_spaces[1][::DOWNSAMPLE], empty_spaces[0][::DOWNSAMPLE], output_name, image_plot)


def plot_results(X, Y, output_name, im):
  # Given the resulting weights and their coordinates, plots the heat map.

  # Normalize likelihoods by max likelihood
  max_weight = np.max(im[1])
  min_weight = np.min(im[1])
  im[1] /= max_weight

  print "MAX WEIGHT"
  print max_weight

  print "MIN WEIGHT"
  print min_weight

 
  # Image has been dealt with as 2 stacked matrices.
  # Perform a rollaxis to convert from (channels, W, H) to (W, H, channels)
  im = np.rollaxis(im, 0, 3)
  print im.shape

  # Plot map
  implot = plt.imshow(im[:,:,0], cmap="Greys")

  # Replace 0s with NaN for plotting
  particle_plot = im[:,:,1]
  particle_plot[particle_plot == 0] = np.nan
  implot = plt.imshow(im[:,:,1], cmap="cool_r")

  plt.savefig(output_name, dpi=500)




if __name__ == "__main__":
  main()

