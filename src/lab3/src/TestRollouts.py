#!/usr/bin/env python


import matplotlib.pyplot as plt
import time
import os
import sys
import rospy
import rosbag
import numpy as np
import scipy.signal
import utils as Utils

import torch
import torch.utils.data
from torch.autograd import Variable



SPEED_TO_ERPM_OFFSET     = 0.0
SPEED_TO_ERPM_GAIN       = 4614.0
STEERING_TO_SERVO_OFFSET = 0.5304
STEERING_TO_SERVO_GAIN   = -1.2135

INPUT_SIZE=8
OUTPUT_SIZE=3
DATA_SIZE=6


def main():

    if len(sys.argv) < 2:
        print "Usage: python TestTrainer.py <model.th>"
        sys.exit()


    print "Generating Saved Model..."
    with open(sys.argv[1]) as f:
        model = torch.load(f).cuda()
   
    model.eval() 
    print "\nLoaded!\n\nTesting model...\n"

    print "N = 10"
    N = 10

    # Still (v = 0, st = 0)
    plot_rollout(model, N, "Still", 'm-')

    # Forward (v = v_max, st = 0)
    plot_rollout(model, N, "Forward", 'k-', velocity=0.7)

    # Backward (v = v_max, st = 0)
    plot_rollout(model, N, "Backward", 'r-', velocity=-0.7)

    # Left (v = v_max, delta = delta_max)
    plot_rollout(model, N, "Left", 'g-', velocity=0.7, delta=0.34)

    # Right (v = v_max, delta = delta_max)
    plot_rollout(model, N, "Right", 'b-', velocity=0.7, delta=-0.34)



# The following are functions meant for debugging and sanity checking your
# model. You should use these and / or design your own testing tools.
# test_model starts at [0,0,0]; you can specify a control to be applied and the
# rollout() function will use that control for N timesteps.
# i.e. a velocity value of 0.7 should drive the car to a positive x value.
def rollout(m, nn_input, N):
    pose = torch.zeros(3).cuda()
    poses = []
    for i in range(N):
        out = m(Variable(nn_input))
        pose.add_(out.data)
        # Wrap pi
        if pose[2] > 3.14:
            pose[2] -= 3.14
        if pose[2] < -3.14:
            pose[2] += 3.14
        nn_input[0] = out.data[0]
        nn_input[1] = out.data[1]
        nn_input[2] = out.data[2]
        nn_input[3] = pose[2]

        pose_value = pose.cpu().numpy()
        poses.append(pose_value)

    return np.array(poses)


def test_model(m, N, dt = 0.1):
    cos, v, st = 4, 5, 6
    s = INPUT_SIZE
    print("Nothing")
    nn_input = torch.zeros(s).cuda()
    nn_input[cos] = 1.0
    nn_input[7] = dt
    rollout(m, nn_input, N)

    print("Forward")
    nn_input = torch.zeros(s).cuda()
    nn_input[cos] = 1.0
    nn_input[v] = 0.7 #1.0
    nn_input[7] = dt
    rollout(m, nn_input, N)


def perform_rollout(m, N, dt=0.1, velocity=0.0, delta=0.0):
    cos, v, st = 4, 5, 6
    s = INPUT_SIZE
    nn_input = torch.zeros(s).cuda()
    nn_input[cos] = 1.0
    nn_input[v] = velocity
    nn_input[st] = delta
    nn_input[7] = dt

    return rollout(m, nn_input, N)


def plot_rollout(model, N, direction, marker, velocity=0.0, delta=0.0):
    if not os.path.exists("plots"):
        os.mkdir("plots")

    axes = plt.gca()
    axes.set_xlim([-1, 1])
    axes.set_ylim([-1, 1])
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    
    title = direction + " Rollout: v = {}, delta = {}".format(velocity, delta)
    file_name = direction + ".png"

    rollout = perform_rollout(model, N, velocity=velocity, delta=delta)
    x_r = rollout[:, 0]
    y_r = rollout[:, 1]
    plt.plot(x_r, y_r, marker)
    plt.title(title)
    plt.savefig(os.path.join("plots", file_name), dpi=300)
    plt.clf()



if __name__ == "__main__":
    main()


