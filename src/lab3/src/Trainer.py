#!/usr/bin/env python

import time
import os
import pickle
import sys
import rospy
import rosbag
import numpy as np
import scipy.signal
import utils as Utils

import torch
import torch.utils.data
from torch.autograd import Variable

from model import MotionModel


SPEED_TO_ERPM_OFFSET     = 0.0
SPEED_TO_ERPM_GAIN       = 4614.0
STEERING_TO_SERVO_OFFSET = 0.5304
STEERING_TO_SERVO_GAIN   = -1.2135

if len(sys.argv) < 2:
    print('Input a bag file from command line')
    print('Input a bag file from command line')
    print('Input a bag file from command line')
    print('Input a bag file from command line')
    print('Input a bag file from command line')
    sys.exit()
bag = rosbag.Bag(sys.argv[1])
tandt = bag.get_type_and_topic_info()
t1='/vesc/sensors/core'
t2='/vesc/sensors/servo_position_command'
t3='/pf/ta/viz/inferred_pose'
topics = [t1,t2,t3]
min_datas = tandt[1][t3][1] # number of t3 messages is less than t1, t2

INPUT_SIZE=8
OUTPUT_SIZE=3
DATA_SIZE=6


BATCH_SIZE = 5000

raw_datas = np.zeros((min_datas,DATA_SIZE))

last_servo, last_vel = 0.0, 0.0
n_servo, n_vel = 0, 0
idx=0


# The following for-loop cycles through the bag file and averages all control
# inputs until an inferred_pose from the particle filter is recieved. We then
# save that data into a buffer for later processing.
# You should experiment with additional data streams to see if your model
# performance improves.
pickled_data = "pickled_data.pkl"
if not os.path.isfile(pickled_data):
    for topic, msg, t in bag.read_messages(topics=topics):
        if topic == t1:
            last_vel   += (msg.state.speed - SPEED_TO_ERPM_OFFSET) / SPEED_TO_ERPM_GAIN
            n_vel += 1
        elif topic == t2:
            last_servo += (msg.data - STEERING_TO_SERVO_OFFSET) / STEERING_TO_SERVO_GAIN
            n_servo += 1
        elif topic == t3 and n_vel > 0 and n_servo > 0:
            timenow = msg.header.stamp
            last_t = timenow.to_sec()
            last_vel /= n_vel
            last_servo /= n_servo
            orientation = Utils.quaternion_to_angle(msg.pose.orientation)
            data = np.array([msg.pose.position.x,
                             msg.pose.position.y,
                             orientation,
                             last_vel,
                             last_servo,
                             last_t])
            raw_datas[idx,:] = data
            last_vel = 0.0
            last_servo = 0.0
            n_vel = 0
            n_servo = 0
            idx = idx+1
            if idx % 1000==0:
                print('.')
    bag.close()

    # Pre-process the data to remove outliers, filter for smoothness, and calculate
    # values not directly measured by sensors

    # Note:
    # Neural networks and other machine learning methods would prefer terms to be
    # equally weighted, or in approximately the same range of values. Here, we can
    # keep the range of values to be between -1 and 1, but any data manipulation we
    # do here from raw values to our model input we will also need to do in our
    # MPPI code.

    # We have collected:
    # raw_datas = [ x, y, theta, v, delta, time]
    # We want to have:
    # x_datas[i,  :] = [x_dot, y_dot, theta_dot, sin(theta), cos(theta), v, delta, dt]
    # y_datas[i-1,:] = [x_dot, y_dot, theta_dot ]

    raw_datas = raw_datas[:idx, :] # Clip to only data found from bag file
    raw_datas = raw_datas[ np.abs(raw_datas[:,3]) < 0.75 ] # discard bad controls
    raw_datas = raw_datas[ np.abs(raw_datas[:,4]) < 0.36 ] # discard bad controls

    with open(pickled_data, "wb") as p:
        pickle.dump(raw_datas, p)
else:
    with open(pickled_data, "r") as p:
        raw_datas = pickle.load(p)

x_datas = np.zeros( (raw_datas.shape[0], INPUT_SIZE) )
y_datas = np.zeros( (raw_datas.shape[0], OUTPUT_SIZE) )

x_dot = np.diff(raw_datas[:, 0])
y_dot = np.diff(raw_datas[:, 1])
theta_dot = np.diff(raw_datas[:, 2])

# Start from 1st column to align with (x_t - x_{t - 1})
sin_theta = np.sin(raw_datas[:, 2][1:])
cos_theta = np.cos(raw_datas[:, 2][1:])

v = raw_datas[:, 3][1:]
delta = raw_datas[:, 4][1:]
dt = np.diff(raw_datas[:,5])

# Set x, y values
x_datas = np.array([ x_dot[:-1], y_dot[:-1], theta_dot[:-1], sin_theta[:-1], cos_theta[:-1], v[1:], delta[1:], dt[:-1] ]).transpose()
y_datas = np.array([ x_dot[1:], y_dot[1:], theta_dot[1:] ]).transpose()

print x_datas.shape, (raw_datas.shape[0], INPUT_SIZE)
print y_datas.shape, (raw_datas.shape[0], OUTPUT_SIZE)


# TODO
# It is critical we properly handle theta-rollover: 
# as -pi < theta < pi, theta_dot can be > pi, so we have to handle those
# cases to keep theta_dot also between -pi and pi
# (originally  named 'pose_dot')

# Handle positive pi cases
gt_x = x_datas[:,2] > np.pi
x_datas[gt_x, 2] = x_datas[gt_x, 2] - 2*np.pi

gt_y = y_datas[:,2] > np.pi
y_datas[gt_y, 2] = y_datas[gt_y, 2] - 2*np.pi

# Handle negative pi cases
gt_x = x_datas[:,2] < -np.pi
x_datas[gt_x, 2] = x_datas[gt_x, 2] + 2*np.pi

gt_y = y_datas[:,2] < -np.pi
y_datas[gt_y, 2] = y_datas[gt_y, 2] + 2*np.pi



# TODO
# Some raw values from sensors / particle filter may be noisy. It is safe to
# filter the raw values to make them more well behaved. We recommend something
# like a Savitzky-Golay filter. You should confirm visually (by plotting) that
# your chosen smoother works as intended.
# An example of what this may look like is in the homework document.

# Filter xs (window / poly sizes as parameters?)
x_datas[:, 0] = scipy.signal.savgol_filter(x_datas[:, 0], 11, 3)
y_datas[:, 0] = scipy.signal.savgol_filter(y_datas[:, 0], 11, 3)

# Filter ys
x_datas[:, 1] = scipy.signal.savgol_filter(x_datas[:, 1], 11, 3)
y_datas[:, 1] = scipy.signal.savgol_filter(y_datas[:, 1], 11, 3)

# Filter thetas
x_datas[:, 2] = scipy.signal.savgol_filter(x_datas[:, 2], 11, 3)
y_datas[:, 2] = scipy.signal.savgol_filter(y_datas[:, 2], 11, 3)


# Convince yourself that input/output values are not strange
print("Xdot  ", np.min(x_datas[:,0]), np.max(x_datas[:,0]))
print("Ydot  ", np.min(x_datas[:,1]), np.max(x_datas[:,1]))
print("Tdot  ", np.min(x_datas[:,2]), np.max(x_datas[:,2]))
print("sin   ", np.min(x_datas[:,3]), np.max(x_datas[:,3]))
print("cos   ", np.min(x_datas[:,4]), np.max(x_datas[:,4]))
print("vel   ", np.min(x_datas[:,5]), np.max(x_datas[:,5]))
print("delt  ", np.min(x_datas[:,6]), np.max(x_datas[:,6]))
print("dt    ", np.min(x_datas[:,7]), np.max(x_datas[:,7]))
print()
print("y Xdot", np.min(y_datas[:,0]), np.max(y_datas[:,0]))
print("y Ydot", np.min(y_datas[:,1]), np.max(y_datas[:,1]))
print("y Tdot", np.min(y_datas[:,2]), np.max(y_datas[:,2]))

######### NN stuff
dtype = torch.cuda.FloatTensor
D_in, H, D_out = INPUT_SIZE, 64, OUTPUT_SIZE

# TODO
# specify your neural network (or other) model here.
# model = torch

model = MotionModel(D_in, H, H // 2, D_out).cuda()  # Remove cuda for CPU training
# loss_fn = torch.nn.MSELoss(size_average=False)
loss_fn = torch.nn.SmoothL1Loss(size_average=False)
learning_rate = 1e-5
opt = torch.optim.Adam(model.parameters(), lr=learning_rate)


num_samples = x_datas.shape[0]
rand_idx = np.random.permutation(num_samples)
x_d = x_datas[rand_idx,:]
y_d = y_datas[rand_idx,:]
split = int(0.9*num_samples)
x_tr = x_d[:split]
y_tr = y_d[:split]
x_tt = x_d[split:]
y_tt = y_d[split:]

x = torch.from_numpy(x_tr.astype('float32')).type(dtype)
y = torch.from_numpy(y_tr.astype('float32')).type(dtype)
x_val = torch.from_numpy(x_tt.astype('float32')).type(dtype)
y_val = torch.from_numpy(y_tt.astype('float32')).type(dtype)

# Shuffle the data after every pass.
def shuffleData():
    num_samples = BATCH_SIZE
    rand_idx = np.random.permutation(num_samples)
    x_s = x[rand_idx,:]
    y_s = y[rand_idx,:]
    return x_s, y_s


def doTraining(model, filename, optimizer, N=5000):
    X_AVG_ERR = 0
    Y_AVG_ERR = 0
    THETA_AVG_ERR = 0

    x_s, y_s = shuffleData()  # Random mini-batch
    for t in range(N):
        y_pred = model(Variable(x_s))
        loss = loss_fn(y_pred, Variable(y_s, requires_grad=False))
        if t % 50 == 0:
            val = model(Variable(x_val))
            vloss = loss_fn(val, Variable(y_val, requires_grad=False))
            print(t, loss.data[0]/x.shape[0], vloss.data[0]/x_val.shape[0])

        # Collect error
        raw_error = torch.abs(y_pred.data - y_s)
        X_AVG_ERR += torch.sum(raw_error[:, 0])
        Y_AVG_ERR += torch.sum(raw_error[:, 1])
        THETA_AVG_ERR += torch.sum(raw_error[:, 2])

        optimizer.zero_grad() # clear out old computed gradients
        loss.backward()       # apply the loss function backprop
        optimizer.step()      # take a gradient step for model's parameters

    torch.save(model, filename)

    num_seen = N * BATCH_SIZE
    print("final avg error:", X_AVG_ERR / num_seen, Y_AVG_ERR / num_seen, THETA_AVG_ERR / num_seen)


if len(sys.argv) > 2:
    doTraining(model, sys.argv[2], opt, N=22000)


# The following are functions meant for debugging and sanity checking your
# model. You should use these and / or design your own testing tools.
# test_model starts at [0,0,0]; you can specify a control to be applied and the
# rollout() function will use that control for N timesteps.
# i.e. a velocity value of 0.7 should drive the car to a positive x value.
def rollout(m, nn_input, N):
    pose = torch.zeros(3).cuda()
    print(pose.cpu().numpy())
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
        print(pose.cpu().numpy())

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

