import sys

import cv2
import matplotlib.pyplot as plt

from Utils import csv_to_configs


def main():

  occupied = (205, 205, 205)
  radius = int(sys.argv[1])
  map_pgm = sys.argv[2]
  waypoints_csv = sys.argv[3]
  configs = csv_to_configs(waypoints_csv)

  print configs

  # Superimpose the bad waypoints onto the map (create holes where they exist)
  map_img = cv2.imread(map_pgm)
  map_height = map_img.shape[0]
  map_width = map_img.shape[1]

  def valid_pt(x, y):
    return x >= 0 and x < map_width and y >= 0 and y < map_height

  for config in configs:
    x, y = config
    for xi in range(x - radius, x + radius):
      for yi in range(y - radius, y + radius):
        if valid_pt(xi, yi):
          cv2.circle(map_img, (xi, yi), radius, occupied, -1)  # Black holes to avoid

  # Save as 'original_name_radiusX.pgm'
  img_name = map_pgm.split(".")[0] + "_radius" + str(radius) + ".pgm"
  print img_name

  cv2.imwrite(img_name, map_img)


if __name__ == "__main__":
  if len(sys.argv) < 4:
    print "Usage: rosrun lab4 ObstacleCreater.py <radius> <Map PGM> <Waypoint CSV>"
    sys.exit()

  main()

