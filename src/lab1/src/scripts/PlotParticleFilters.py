#!/usr/bin/env python
"""

"""

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

num_particles = [100, 200, 500, 1000, 4000, 8000]
low_var_times = [
  6.59771587538e-05,
  0.000108760523509,
  0.000252170562744,
  0.000455241454275,
  0.00194236210414,
  0.00366587109036
]

low_var_variance = [
  [ 0.05899797, 0.05786036, 7.65847711 ],
  [ 0.03278461, 0.04185458, 6.08363841 ],
  [ 0.04622368, 0.04598303, 8.19769421 ],
  [ 0.05767742, 0.05782820, 7.56220885 ],
  [ 0.04145072, 0.03668040,  7.48811326 ],
  [ 0.03603577, 0.03424633, 5.03043783 ]
]

naiive_times = [
  6.01159201728e-05,
  6.83935426122e-05,
  9.65891584838e-05,
  0.00013891152576,
  0.000425554238833,
  0.00089694721864,
]

naiive_variance = [
  [ 0.02301288, 0.0584495, 8.11427301],
  [ 0.03187733, 0.03521201, 8.50912309],
  [ 0.05496104, 0.06825320, 7.03151589],
  [ 0.03711593, 0.04339008, 8.25223125],
  [ 0.03272507, 0.03343918, 6.16988153],
  [ 0.05736139, 0.04914861, 6.50262097]
]

fig1 = plt.figure(1)
fig1.suptitle('Particle Resampling Compute Time', fontsize=20)
plt.plot(num_particles, low_var_times)
plt.plot(num_particles, naiive_times)
plt.xlabel('# Particles')
plt.ylabel('Compute Time (secs)')

plt.legend(('low_variance', 'naiive'))
plt.show()

