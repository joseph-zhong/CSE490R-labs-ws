
from PIL import Image
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

THETA_DISCRETIZATION = 15
CONVERSION = 2 * np.pi / THETA_DISCRETIZATION

RESOLUTION = 0.0
OFFSET_X = 0.0
OFFSET_Y = 0.0
OFFSET_THETA = 0.0

DOWNSAMPLE = 1  # Rate to downsample plot
COUNT = 0  # Number of points plotted

def main():

  if len(sys.argv) < 3:
    print "Usage: python HeatMap.py <laser_scanX.bag> <map_img>"
    sys.exit()

  print "Hello!"

  laser_bag_path = sys.argv[1]
  image_path = sys.argv[2]

  # Get the map
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

  # Collect number of pixels
  map_shape = (map_msg.info.width, map_msg.info.height)
  map_data = np.array(map_msg.data).reshape(map_shape)

  # For plotting
  map_data = map_data == 0
  empty_spaces = np.where(map_data)

#  for i in range(len(empty_spaces[0])):
#      print map_data[empty_spaces[0][i]][empty_spaces[1][i]]

  # Create particles from an expanded set of the empty space
  # Reserve a copy of non-expanded space for plotting
  # Convert from map to world
  Y = empty_spaces[0] * RESOLUTION + OFFSET_Y
  X = empty_spaces[1] * RESOLUTION + OFFSET_X
  expanded_spaces = [list(xy) for xy in zip(X, Y)]

  # map_to_world = lambda p: [p[0] * RESOLUTION + OFFSET_X, p[1] * RESOLUTION + OFFSET_Y]
  # expanded_spaces = list(map(map_to_world, expanded_spaces))
  expanded_spaces = expanded_spaces[::DOWNSAMPLE]

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

  # Take the max
  resulting_weights = map(lambda x: np.max(x), calculated_weights)

  print "# OF PARTICLES AND WEIGHTS --------------"
  print len(particles)
  print len(resulting_weights)
  print len(empty_spaces[0])

  plot_results(empty_spaces[1][::DOWNSAMPLE], empty_spaces[0][::DOWNSAMPLE], resulting_weights, map_data)


def plot_results(X, Y, weights, im):
  # Given the resulting weights and their coordinates, plots the heat map.
  max_weight = max(weights)
  print "MAX WEIGHT"
  print max_weight

  print X
  print Y


  nested_weights = np.array([[1.0, 0.0, 0.0, x / max_weight] for x in weights])
  print "MIN WEIGHT"
  print min(weights)

  def plot_single_point(pt):
    global COUNT

    x = pt[0]
    y = pt[1]
    w = pt[2]

    print "\rIteration: {}".format(COUNT),
    sys.stdout.flush()
    COUNT += 1
    plt.scatter([x], [y], c=(0.0, 0.0, 1.0, w / max_weight), lw=0)
    return pt

  implot = plt.imshow(im, cmap="hot")

  ax = plt.gca()
  ax.xaxis.tick_top()

#  np.apply_along_axis(plot_single_point, 1, points_with_opacity)
  plt.scatter(X, Y, c=nested_weights, lw=0)

  plt.show()




if __name__ == "__main__":
  main()

