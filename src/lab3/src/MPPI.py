#!/usr/bin/env python

import time
import sys
import rospy
import rosbag
import numpy as np
import utils as Utils

import torch
import torch.utils.data
from torch.autograd import Variable

from nav_msgs.srv import GetMap
from ackermann_msgs.msg import AckermannDriveStamped
from vesc_msgs.msg import VescStateStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseArray, PoseWithCovarianceStamped, PointStamped

MAX_ANGLE = 0.34
MAX_VEL = 2.0
T = 30
K = 2000
C = 100000
STEERING_SIGMA = 0.15  # These values will need to be tuned
VELOCITY_SIGMA = 0.2
_LAMBDA = 0.1
THETA_WEIGHT = 1.0

class MPPIController:

  def __init__(self):
    self.SPEED_TO_ERPM_OFFSET = float(rospy.get_param("/vesc/speed_to_erpm_offset", 0.0))
    self.SPEED_TO_ERPM_GAIN = float(rospy.get_param("/vesc/speed_to_erpm_gain", 4614.0))
    self.STEERING_TO_SERVO_OFFSET = float(rospy.get_param("/vesc/steering_angle_to_servo_offset", 0.5304))
    self.STEERING_TO_SERVO_GAIN = float(rospy.get_param("/vesc/steering_angle_to_servo_gain", -1.2135))
    self.CAR_LENGTH = 0.33

    self.last_pose = None

    self.goal = None  # Lets keep track of the goal pose (world frame) over time
    self.lasttime = None

    # PyTorch / GPU data configuration
    # TODO
    # you should pre-allocate GPU memory when you can, and re-use it when
    # possible for arrays storing your controls or calculated MPPI costs, etc
    self.controls = torch.cuda.FloatTensor(2, T).zero_()
    self.rollouts = torch.cuda.FloatTensor(T, K, 3).zero_()

    self.nn_input = torch.cuda.FloatTensor(8).zero_()


    model_name = rospy.get_param("~nn_model", "myneuralnetisbestneuralnet.pt")
    self.model = torch.load(model_name)
    self.model.cuda()  # tell torch to run the network on the GPU
    self.dtype = torch.cuda.FloatTensor
    print("Loading:", model_name)
    print("Model:\n", self.model)
    print("Torch Datatype:", self.dtype)

    # control outputs
    self.msgid = 0

    # visualization paramters
    self.num_viz_paths = 40
    if K < self.num_viz_paths:
      self.num_viz_paths = K

    # We will publish control messages and a way to visualize a subset of our
    # rollouts, much like the particle filter
    self.ctrl_pub = rospy.Publisher(rospy.get_param("~ctrl_topic",
      "/vesc/high_level/ackermann_cmd_mux/input/nav_0"),
                                    AckermannDriveStamped, queue_size=2)
    self.path_pub = rospy.Publisher("/mppi/paths", Path, queue_size=self.num_viz_paths)

    # Use the 'static_map' service (launched by MapServer.launch) to get the map
    map_service_name = rospy.get_param("~static_map", "static_map")
    print("Getting map from service: ", map_service_name)
    rospy.wait_for_service(map_service_name)
    map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map  # The map, will get passed to init of sensor model
    self.map_info = map_msg.info  # Save info about map for later use
    print("Map Information:\n", self.map_info)

    # Create numpy array representing map for later use
    self.map_height = map_msg.info.height
    self.map_width = map_msg.info.width
    array_255 = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
    self.permissible_region = np.zeros_like(array_255, dtype=bool)
    self.permissible_region[array_255 == 0] = 1  # Numpy array of dimension (map_msg.info.height, map_msg.info.width),
    # With values 0: not permissible, 1: permissible
    self.permissible_region = np.negative(self.permissible_region)  # 0 is permissible, 1 is not
    self.permissible_region = self.permissible_region.astype(int)
    self.permissible_region = torch.cuda.IntTensor(self.permissible_region)
    print "The size of the permissible region is:", self.permissible_region.shape

    print("Making callbacks")
    self.goal_sub = rospy.Subscriber("/move_base_simple/goal",
                                     PoseStamped, self.clicked_goal_cb, queue_size=1)
    self.pose_sub = rospy.Subscriber("/pf/viz/inferred_pose",
                                     PoseStamped, self.mppi_cb, queue_size=1)

  # TODO
  # You may want to debug your bounds checking code here, by clicking on a part
  # of the map and convincing yourself that you are correctly mapping the
  # click, and thus the goal pose, to accessible places in the map
  def clicked_goal_cb(self, msg):
    self.goal = torch.cuda.FloatTensor([msg.pose.position.x,
                          msg.pose.position.y,
                          Utils.quaternion_to_angle(msg.pose.orientation)])
    print("Current Pose: ", self.last_pose)
    print("SETTING Goal: ", self.goal)

  def running_cost(self, pose, goal, ctrl, noise):
    # TODO
    # This cost function drives the behavior of the car. You want to specify a
    # cost function that penalizes behavior that is bad with high cost, and
    # encourages good behavior with low cost.
    # We have split up the cost function for you to a) get the car to the goal
    # b) avoid driving into walls and c) the MPPI control penalty to stay
    # smooth
    # You should feel free to explore other terms to get better or unique
    # behavior

    # PRE-CONDITION: Expected pose and goal shape (1, K)
    pose_cost = (pose - goal) ** 2
    pose_cost[:, 2] *= THETA_WEIGHT
    pose_cost = torch.sum(pose_cost, 1)

    # Clone pose for world to map conversion.
    pose_copy = pose.clone()
    Utils.world_to_map(pose_copy, self.map_info)

    # Bounds checking: Convert coordinates to ints before indexing
    y = pose_copy[:, 1].long()
    x = pose_copy[:, 0].long()
    bounds_check = self.permissible_region[y, x] * C
    bounds_check = bounds_check.float().squeeze(0)

    # Control cost using steering and velocity sigmas.
    ctrl = ctrl.unsqueeze(1)  # Unsqueeze from (2,) to (2,1) for matrix mult.
    noise_ctrl_mm = noise.mm(ctrl)
    ctrl_cost = noise_ctrl_mm * (_LAMBDA / STEERING_SIGMA)
    ctrl_cost += noise_ctrl_mm * (_LAMBDA / VELOCITY_SIGMA)
    ctrl_cost = ctrl_cost.squeeze()  # Squeeze from (K, 1) to (K,)

    return pose_cost + ctrl_cost  # Expected (K,)

  def mppi(self, curr_pose, init_input):
    t0 = time.time()
    # Network input can be:
    #   0    1       2          3           4        5      6   7
    # xdot, ydot, thetadot, sin(theta), cos(theta), vel, delta, dt

    # MPPI should
    # generate noise according to sigma
    # combine that noise with your central control sequence
    # Perform rollouts with those controls from your current pose
    # Calculate costs for each of K trajectories
    # Perform the MPPI weighting on your calculatd costs
    # Scale the added noise by the weighting and add to your control sequence
    # Apply the first control values, and shift your control trajectory

    # Notes:
    # MPPI can be assisted by carefully choosing lambda, and sigma
    # It is advisable to clamp the control values to be within the feasible range
    # of controls sent to the Vesc
    # Your code should account for theta being between -pi and pi. This is
    # important.
    # The more code that uses pytorch's cuda abilities, the better; every line in
    # python will slow down the control calculations. You should be able to keep a
    # reasonable amount of calculations done (T = 40, K = 2000) within the 100ms
    # between inferred-poses from the particle filter.
    vel_noise = np.random.normal(loc=0.0, scale=VELOCITY_SIGMA, size=(K, 1, T))
    delta_noise = np.random.normal(loc=0.0, scale=STEERING_SIGMA, size=(K, 1, T))
    noise = torch.cuda.FloatTensor(np.concatenate((vel_noise, delta_noise), axis=1))

    # Bound the noisy controls to car's extremes
    noisy_controls = noise + self.controls
    noisy_controls[:, 0, :] = torch.clamp(noisy_controls[:, 0, :], -MAX_VEL, MAX_VEL)
    noisy_controls[:, 1, :] = torch.clamp(noisy_controls[:, 1, :], -MAX_ANGLE, MAX_ANGLE)

    noisy_controls = torch.transpose(noisy_controls, 0, 2)

    model_input = torch.cuda.FloatTensor(K, 8).zero_()  # TODO: Allocate in init.
    model_input += init_input
    cost = torch.cuda.FloatTensor(K, 1).zero_()

    self.rollouts *= 0
    for t in xrange(T):
      model_input[:, 5] = noisy_controls[t, 0, :]
      model_input[:, 6] = noisy_controls[t, 1, :]

      # Perform a forward pass using perturbed controls
      rollout_step_delta = self.model(Variable(model_input))

      # Extract the tensor
      rollout_step_delta = rollout_step_delta.data

      if t == 0:
        self.rollouts[t] = rollout_step_delta + curr_pose
      else:
        self.rollouts[t] = rollout_step_delta + self.rollouts[t - 1]


      model_input[:, 0] = rollout_step_delta[:, 0]
      model_input[:, 1] = rollout_step_delta[:, 1]
      model_input[:, 2] = rollout_step_delta[:, 2]
      theta = self.rollouts[t, :, 2].clone()
      model_input[:, 3] = np.sin(theta)
      model_input[:, 4] = np.cos(theta)
      cost += self.running_cost(self.rollouts[t], self.goal, self.controls[:, t], noise[:, :, t])


    beta = torch.min(cost)
    eta = torch.sum(torch.exp((cost - beta) / _LAMBDA * -1))

    weights = torch.exp((cost - beta) / _LAMBDA * -1) / eta
    weights = weights.squeeze()  # Squeeze allows matrix multiply

    # Add weighted noise to current controls.
    noise = torch.transpose(noise, 0, 2)  # (T, 2, K) allows looping over time steps
    for t in xrange(T):
      vel_noise = torch.cuda.FloatTensor(noise[t][0])
      steer_noise = torch.cuda.FloatTensor(noise[t][1])

      vel_weighted_noise = weights.dot(vel_noise)
      steer_weighted_noise = weights.dot(steer_noise)

      self.controls[0][t] += vel_weighted_noise
      self.controls[1][t] += steer_weighted_noise


    print("MPPI: %4.5f ms" % ((time.time() - t0) * 1000.0))

    run_ctrl = self.controls[:, 0]
    poses = self.rollouts.transpose(0, 1)  # Input to particle_to_posestamped should be (K, T, 3)

    return run_ctrl, poses

  def mppi_cb(self, msg):
    # print("callback")
    if self.last_pose is None:
      self.last_pose = torch.cuda.FloatTensor([msg.pose.position.x,
                                 msg.pose.position.y,
                                 Utils.quaternion_to_angle(msg.pose.orientation)])
      # Default: initial goal to be where the car is when MPPI node is
      # initialized
      self.goal = self.last_pose
      self.lasttime = msg.header.stamp.to_sec()
      return

    theta = Utils.quaternion_to_angle(msg.pose.orientation)
    curr_pose = torch.cuda.FloatTensor([msg.pose.position.x,
                          msg.pose.position.y,
                          theta])

    pose_dot = curr_pose - self.last_pose  # get state
    self.last_pose = curr_pose

    timenow = msg.header.stamp.to_sec()
    dt = timenow - self.lasttime
    self.lasttime = timenow
    self.nn_input[0] = pose_dot[0]
    self.nn_input[1] = pose_dot[1]
    self.nn_input[2] = pose_dot[2]
    self.nn_input[3] = np.sin(theta)
    self.nn_input[4] = np.cos(theta)
    self.nn_input[5] = 0.0
    self.nn_input[6] = 0.0
    self.nn_input[7] = dt

    run_ctrl, poses = self.mppi(curr_pose, self.nn_input)

    self.send_controls(run_ctrl[0], run_ctrl[1])

    # Shifting past controls, duplicating previous final controls.
    temp = self.controls[:, -1].clone()
    self.controls[:, :-1] = self.controls[:, 1:]
    self.controls[:, -1] = temp

    self.visualize(poses)

  def send_controls(self, speed, steer):
    print("Speed:", speed, "Steering:", steer)
    ctrlmsg = AckermannDriveStamped()
    ctrlmsg.header.seq = self.msgid
    ctrlmsg.drive.steering_angle = steer
    ctrlmsg.drive.speed = speed
    self.ctrl_pub.publish(ctrlmsg)
    self.msgid += 1

  # Publish some paths to RVIZ to visualize rollouts
  def visualize(self, poses):
    if self.path_pub.get_num_connections() > 0:
      frame_id = 'map'
      start = time.clock()
      for i in range(0, self.num_viz_paths):
        pa = Path()
        pa.header = Utils.make_header(frame_id)
        pa.poses = map(Utils.particle_to_posestamped, poses[i, :, :], [frame_id] * T)
        self.path_pub.publish(pa)

      print "vis time:", time.clock() - start


def test_MPPI(mp, N, goal=np.array([0., 0., 0.])):
  init_input = np.array([0., 0., 0., 0., 1., 0., 0., 0.])
  pose = np.array([0., 0., 0.])
  mp.goal = goal
  print("Start:", pose)
  mp.ctrl.zero_()
  last_pose = np.array([0., 0., 0.])
  for i in range(0, N):
    # ROLLOUT your MPPI function to go from a known location to a specified
    # goal pose. Convince yourself that it works.

    print("Now:", pose)
  print("End:", pose)


if __name__ == '__main__':


  # run with ROS
  rospy.init_node("mppi_control", anonymous=True) # Initialize the node
  mp = MPPIController()
  rospy.spin()

  # test & DEBUG
  # mp = MPPIController()
  # test_MPPI(mp, 10, np.array([0., 0., 0.])))

