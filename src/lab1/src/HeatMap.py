
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

THETA_DISCRETIZATION = 112
CONVERSION = 2 * np.pi / THETA_DISCRETIZATION

RESOLUTION = 0.0
OFFSET_X = 0.0
OFFSET_Y = 0.0

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
  map_shape =  (map_msg.info.height, map_msg.info.width)
  map_data = np.array(map_msg.data).reshape(map_shape)
  empty_spaces = np.where(map_data == 0)

  # For plotting
  map_data = map_data == 0
 
  # Create particles from an expanded set of the empty space
  # Reserve a copy of non-expanded space for plotting
  empty_spaces = [list(xy) for xy in zip(empty_spaces[0], empty_spaces[1])]
  expanded_spaces = np.copy(empty_spaces)

  # Convert from map to world
  map_to_world = lambda p: [p[0] * RESOLUTION + OFFSET_X, p[1] * RESOLUTION + OFFSET_Y]
  expanded_spaces = list(map(map_to_world, expanded_spaces))
  expanded_spaces = expanded_spaces[::100]

  # Create particles, THETA_DISCRETIZATION thetas per particle
  particles = np.empty((0, 3), dtype=np.float32)
  for i in range(THETA_DISCRETIZATION):
    curr_thetas = np.array([[i * CONVERSION] for x in range(len(expanded_spaces))])
    e = np.append(expanded_spaces, curr_thetas, axis=1)
    particles = np.append(particles, e, axis=0)

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

  plot_results(RESOLUTION, OFFSET_X, OFFSET_Y, empty_spaces[::100], resulting_weights, map_data)


def plot_results(res, off_x, off_y, spaces, weights, im):
  # Given the resulting weights and their coordinates, plots the heat map.
  max_weight =  max(weights)

  nested_weights = np.array([[x] for x in weights])
  points_with_opacity = np.append(spaces, nested_weights, axis=1)

  print points_with_opacity[0:15]

  def plot_single_point(pt):
    x = pt[0]
    y = pt[1]
    w = pt[2]

    plt.scatter([x], [y], c=(0.0, 0.0, 1.0, w / max_weight), lw=0)
    return pt

  implot = plt.imshow(im, cmap="hot")

  np.apply_along_axis(plot_single_point, 1, points_with_opacity)

  plt.show()




if __name__ == "__main__":
  main()

