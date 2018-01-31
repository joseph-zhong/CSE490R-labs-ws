#!/usr/bin/env python

import os

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

realtimes = np.load('realtimes.npy')
rmses = {}

for fname in os.listdir('.'):
  if '00.npy' not in fname:
    continue

  rms = np.load(fname)
  rmses[fname] = rms

for i, c in enumerate(['X', 'Y', 'Theta']):
  particles = []
  for n, rms in rmses.iteritems():
    print n
    fuck = rms[:, i]
    plt.plot(realtimes, fuck)
    particles.append(n.split('.')[0])

  print particles
  print
  # fig = plt.figure(0)
  plt.suptitle('{} RMS Error Over Time'.format(c), fontsize=17)
  plt.legend(list('n={}'.format(l) for l in particles),
    loc='center left', bbox_to_anchor=(1, 0.5))

  plt.xlabel('Rospy Time (s)')
  plt.ylabel('RMS Error')
  plt.show()




