
from PIL import Image
from matplotlib.pyplot import cm
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import sys
import time

from nav_msgs.srv import GetMap
import range_libc
import rosbag
import rospy
from sensor_msgs.msg import LaserScan

from SensorModelHeatMap import SensorModelHeatMap


DOWNSAMPLE = 10  # Rate to downsample plot for data collection
COUNT = 0  # Number of points plotted

def main():

  if len(sys.argv) < 2:
    print "Usage: python HeatMap.py <laser_scanX.bag>"
    sys.exit()

  laser_bag_path = sys.argv[1]

  # Process Bag
  laser_bag = rosbag.Bag(laser_bag_path)
  topic, msg, t = next(laser_bag.read_messages())

  # Get the map
  map_msg_srv = rospy.ServiceProxy("static_map", GetMap)
  try:
    map_msg = map_msg_srv().map
  except rospy.ServiceException as exc:
    raise rospy.ServiceException("Service did not process: request " + str(exc))


  times = []

  # Collect computation time as theta increases.
  progress = "\nDisc. : {} Time taken: {} ------------------\n"
  for i in range(10, 201, 10):
    start = time.time()
    heat_map_construction(i, msg, map_msg)
    end = time.time()
    duration = end - start

    print progress.format(i, duration),
    sys.stdout.flush() 
    
    times.append(duration)

  # Plot the times
  fig = plt.figure()
  ax = fig.add_subplot(111)  
  ax.set_title('Theta Discretization vs. Computation Time')
  ax.set_xlabel('Number of Angles')
  ax.set_ylabel('Computation Time in Seconds')
  ax.scatter(range(10, 201, 10), times)
  
  plt.show()


def heat_map_construction(theta, msg, map_msg):

  THETA_DISCRETIZATION = theta
  CONVERSION = 2 * np.pi / THETA_DISCRETIZATION

  # print "Hello!"

  # print map_msg.info
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
  Y = empty_spaces[0] * RESOLUTION + OFFSET_Y
  X = empty_spaces[1] * RESOLUTION + OFFSET_X
  expanded_spaces = [list(xy) for xy in zip(X, Y)]
  expanded_spaces = expanded_spaces[::DOWNSAMPLE]


  #########################
  ### SENSOR MODEL OBS
  downsampled_angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment * 18, dtype=np.float32)
  downsampled_ranges = np.array(msg.ranges[::18], dtype=np.float32)
  isNan = np.isnan(downsampled_ranges)
  downsampled_ranges[isNan] = msg.range_max
  obs = (downsampled_ranges, downsampled_angles)


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





if __name__ == "__main__":
  main()

