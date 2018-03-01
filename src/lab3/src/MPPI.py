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

import matplotlib.pyplot as plt
from nav_msgs.srv import GetMap
from ackermann_msgs.msg import AckermannDriveStamped
from vesc_msgs.msg import VescStateStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseArray, PoseWithCovarianceStamped, PointStamped

MAX_ANGLE = 0.34
MAX_VEL = 2.0

class MPPIController:


  def __init__(self, C, T, K, velocity_sigma, steering_sigma, _lambda=0.5):
    self.SPEED_TO_ERPM_OFFSET = float(rospy.get_param("/vesc/speed_to_erpm_offset", 0.0))
    self.SPEED_TO_ERPM_GAIN   = float(rospy.get_param("/vesc/speed_to_erpm_gain", 4614.0))
    self.STEERING_TO_SERVO_OFFSET = float(rospy.get_param("/vesc/steering_angle_to_servo_offset", 0.5304))
    self.STEERING_TO_SERVO_GAIN   = float(rospy.get_param("/vesc/steering_angle_to_servo_gain", -1.2135))
    self.CAR_LENGTH = 0.33

    self.last_pose = None

    # MPPI params
    self.C = C
    self.T = T # Length of rollout horizon
    self.K = K # Number of sample rollouts
    self.velocity_sigma = velocity_sigma
    self.steering_sigma = steering_sigma
    self._lambda = _lambda
    self._theta_weight = 0.5

    # Initialize controls
    self.controls = torch.cuda.FloatTensor(2, T).zero_()
    self.goal = None  # Lets keep track of the goal pose (world frame) over time
    self.lasttime = None

    # PyTorch / GPU data configuration
    # TODO
    # you should pre-allocate GPU memory when you can, and re-use it when
    # possible for arrays storing your controls or calculated MPPI costs, etc
    print "Initializing model"
    model_name = rospy.get_param("~nn_model", "")
    self.model = torch.load(model_name)
    self.model.cuda() # tell torch to run the network on the GPU
    self.dtype = torch.cuda.FloatTensor
    print("Loading:", model_name)
    print("Model:\n",self.model)
    print("Torch Datatype:", self.dtype)

    # control outputs
    self.msgid = 0

    # visualization paramters
    self.num_viz_paths = 40
    if self.K < self.num_viz_paths:
        self.num_viz_paths = self.K

    # We will publish control messages and a way to visualize a subset of our
    # rollouts, much like the particle filter
    self.ctrl_pub = rospy.Publisher(rospy.get_param("~ctrl_topic",
            "/vesc/high_level/ackermann_cmd_mux/input/nav0"),
            AckermannDriveStamped, queue_size=2)
    self.path_pub = rospy.Publisher("/mppi/paths", Path, queue_size=self.num_viz_paths)

    # Use the 'static_map' service (launched by MapServer.launch) to get the map
    map_service_name = rospy.get_param("~static_map", "static_map")
    print("Getting map from service: ", map_service_name)
    rospy.wait_for_service(map_service_name)
    map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map # The map, will get passed to init of sensor model
    self.map_info = map_msg.info # Save info about map for later use    
    print("Map Information:\n",self.map_info)

    # Create numpy array representing map for later use
    self.map_height = map_msg.info.height
    self.map_width = map_msg.info.width
    array_255 = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
    self.permissible_region = np.zeros_like(array_255, dtype=bool)

    # Numpy array of dimension (map_msg.info.height, map_msg.info.width),
    # With values 0: not permissible, 1: permissible
    self.permissible_region[array_255 == 0] = 1

    # 0 is permissible, 1 is not
    # self.permissible_region = np.negative(self.permissible_region)
    self.permissible_region = self.permissible_region == 0

    print("Making callbacks")
    self.goal_sub = rospy.Subscriber("/move_base_simple/goal",
            PoseStamped, self.clicked_goal_cb, queue_size=1)
    self.pose_sub  = rospy.Subscriber("/pf/viz/inferred_pose",
            PoseStamped, self.mppi_cb, queue_size=1)
    print "Done Initalizing"

  # TODO
  # You may want to debug your bounds checking code here, by clicking on a part
  # of the map and convincing yourself that you are correctly mapping the
  # click, and thus the goal pose, to accessible places in the map
  def clicked_goal_cb(self, msg):
    self.goal = torch.cuda.FloatTensor([msg.pose.position.x,
                          msg.pose.position.y,
                          Utils.quaternion_to_angle(msg.pose.orientation)])

    clicked_c = np.array([[msg.pose.position.x,
                          msg.pose.position.y,
                          Utils.quaternion_to_angle(msg.pose.orientation)]])
    Utils.world_to_map(clicked_c, self.map_info)
    print "The region is permissible? ", self.permissible_region[clicked_c[:, 1].astype(np.int), clicked_c[:, 0].astype(np.int)]
    print "The inverse region is permissible? ", self.permissible_region[clicked_c[:, 0].astype(np.int), clicked_c[:, 1].astype(np.int)]
    print("Clicked_c ", clicked_c)
    # print np.where(self.permissible_region == 0)
    print("Current Pose: ", self.last_pose)
    print("SETTING Goal: ", self.goal)

  def running_cost(self, pose, goal, ctrl, noise):
    # This cost function drives the behavior of the car. You want to specify a
    # cost function that penalizes behavior that is bad with high cost, and
    # encourages good behavior with low cost.
    # We have split up the cost function for you to a) get the car to the goal
    # b) avoid driving into walls and c) the MPPI control penalty to stay
    # smooth
    # You should feel free to explore other terms to get better or unique
    # behavior

    # REVIEW: We have ignored Q Matrix of scaling factors. It should be size as goal or pose.
    # BROKEN BROKEN BROKEN BROKEN BROKEN BROKEN BROKEN BROKEN BROKEN BROKEN (?)
    # These are supposed to be in world coordinates.
    pose_c = pose.cpu().numpy()
    goal_c = goal.cpu().numpy()
    ctrl_c = ctrl.cpu().numpy()
    noise_c = noise.cpu().numpy()


    # print "goal after world_to_map:", goal
    # goal = torch.cuda.FloatTensor(goal)
    # goal = torch.squeeze(goal)
    # print "goal after torchify + squeeze:", goal

    # Cost between pose and goal
    # pose = pose.cuda()
    # TODO different semantic meanings
    pose_cost = (pose_c - goal_c) ** 2
    pose_cost[:, 2] *= self._theta_weight
    pose_cost = np.sum(pose_cost, 1)

    print "(running_cost) y values max:", np.max(pose_c[:, 1])
    print "(running_cost) x values max:", np.max(pose_c[:, 0])
    print "(running_cost) y values min:", np.min(pose_c[:, 1])
    print "(running_cost) x values min:", np.min(pose_c[:, 0])

    # print "init. pose data:", pose
    Utils.world_to_map(pose_c, self.map_info)

    # print "goal before expanding:", goal
    goal_c = np.expand_dims(goal_c, axis=0)
    Utils.world_to_map(goal_c, self.map_info)
    goal_c = np.squeeze(goal_c)


    # # Remove points that are not on the map.
    # def create_bounds():
    #   coordinates = zip(pose[:, 0].long(), pose[:, 1].long())
    #   bounds = torch.cuda.LongTensor(K)
    #   for i, c in enumerate(coordinates):
    #     if 0 <= c[0] < self.map_height and 0 <= c[1] < self.map_width:
    #       bounds[i] = 1
    #   return bounds
    #
    # def create_bounds_c():
    #   coordinates = zip(pose[:, 0].astype(np.int), pose[:, 1].astype(np.int))
    #   bounds = np.empty(K)
    #   for i, c in enumerate(coordinates):
    #     if 0 <= c[0] < self.map_height and 0 <= c[1] < self.map_width:
    #       bounds[i] = 1
    #   return bounds

    # bounds_within_region = create_bounds()
    # y_bounds = pose_c[:, 0].astype(np.int) * bounds_within_region
    # x_bounds = pose_c[:, 1].astype(np.int) * bounds_within_region
    #
    # print "y values (bounded):", np.max(y_bounds)
    # print "x values (bounded):", np.max(x_bounds)

    # bounds_check = torch.cuda.FloatTensor(self.C * self.permissible_region[y_bounds, x_bounds])
    bounds_check_c = self.C * self.permissible_region[pose_c[:, 1].astype(np.int), pose_c[:, 0].astype(np.int)]
    # ctrl = ctrl.unsqueeze(1)  # Extend from (2,) to (2, 1) to allow matrix multiply
    ctrl_c = np.expand_dims(ctrl_c, 1)
    # ctrl_cost = self._lambda * torch.mm(noise, ctrl) / self.steering_sigma
    ctrl_cost = self._lambda * np.matmul(noise_c, ctrl_c) / self.steering_sigma
    ctrl_cost = ctrl_cost.squeeze()

    print "The number of particles outside the permissible region = ",  np.sum(self.permissible_region[pose_c[:, 1].astype(np.int), pose_c[:, 0].astype(np.int)])
    print "(running_cost) max bounds_check", np.max(bounds_check_c)
    print "(running_cost) max ctrl_cost", np.max(ctrl_cost), "mean ctrl cost", np.mean(ctrl_cost), "min ctrl cost", np.min(ctrl_cost)
    print "(running_cost) max pose_cost", np.max(pose_cost), "mean pose cost", np.mean(pose_cost), "min pose cost", np.min(pose_cost)

    return torch.cuda.FloatTensor(pose_cost + ctrl_cost + bounds_check_c) # Returns vector of shape (K,)

  def mppi(self, curr_pose, init_input):
    """
    :param curr_pose:
    :param init_input: Input to model.
      Network input can be:
        [ xdot, ydot, thetadot, sin(theta), cos(theta), vel, delta, dt ]
    :return run_ctrl [v, d] and poses as [k, t, 3]
    """
    t0 = time.time()

    print "Entering MPPI, init_pose", curr_pose, "init_input max", torch.max(init_input)

    # Generate noise for vel, and delta.
    # REVIEW: Make new sigma for one or the other.
    # min and max the randoms and give them different sigma
    vel_noise = np.random.normal(loc=0.0, scale=self.velocity_sigma, size=(self.K, 1, self.T))
    delta_noise = np.random.normal(loc=0.0, scale=self.steering_sigma, size=(self.K, 1, self.T))
    print "The min of the delta noise is: ", np.min(delta_noise)
    print "The max of the delta noise is: ", np.max(delta_noise)
    print "The min of the vel noise is: ", np.min(vel_noise)
    print "The max of the vel noise is: ", np.max(vel_noise)
    noise = np.concatenate((vel_noise, delta_noise), axis=1)
    py_noise = torch.cuda.FloatTensor(noise)
    # print "Controls before adding to py_noise:", self.controls
    noisy_control = py_noise + self.controls
    # print "Noisy Controls before capping:", noisy_control
    noisy_control[:, 1, :][noisy_control[:, 1, :] >= MAX_ANGLE] = MAX_ANGLE
    noisy_control[:, 1, :][noisy_control[:, 1, :] <= -MAX_ANGLE] = -MAX_ANGLE
    noisy_control[:, 0, :][noisy_control[:, 0, :] >= MAX_VEL] = MAX_VEL
    noisy_control[:, 0, :][noisy_control[:, 0, :] <= -MAX_VEL] = -MAX_VEL
    print "The min of the noisy_control delta is: ", torch.min(noisy_control[:, 1, :])
    print "The max of the noisy_control delta is: ", torch.max(noisy_control[:, 1, :])
    print "The max of the noisy_control vel is: ", torch.max(noisy_control[:, 0, :])
    print "The min of the noisy_control vel is: ", torch.min(noisy_control[:, 0, :])
    print "Noisy Controls after capping:", noisy_control
    print "The current controls are ", self.controls
    # Transpose to become shape (T, 2, K).
    noisy_control = torch.transpose(noisy_control, 0, 2)
    # print "noisy_control", noisy_control.shape
    # print "py_noise", py_noise.shape

    init = torch.cuda.FloatTensor(self.K, 8).zero_()
    th_init_input = torch.cuda.FloatTensor(init_input)
    init_state = init + th_init_input

    xts = torch.cuda.FloatTensor(self.T, self.K, 3).zero_()
    # Perform rollouts with those controls from your current pose
    cost = torch.cuda.FloatTensor(self.K, 1).zero_()

    # Plotting helpers.
    import matplotlib.cm as cm
    x_plot = []
    y_plot = []
    colors = cm.rainbow(np.linspace(0, 1, self.T))
    # from matplotlib import colors
    # import random
    # clist = [i for i in colors.ColorConverter.colors if i != 'w']
    # color = (i for i in random.sample(clist, (len(clist))))
    st_time = time.time()
    for t in xrange(self.T):
      init_state[:, 5] = noisy_control[t, 0, :]
      init_state[:, 6] = noisy_control[t, 1, :]

      # Model was trained on [b, 8] inputs.
      # x_t is output [k, poses]
      # print "Here is a picture of init_state: ", init_state
      s_time = time.time()
      x_t = self.model(Variable(init_state.cuda()))
      print "MPPI model call CPU time", time.time() - s_time
      # print "Here are the rollouts for this t", x_t
      if t == 0:
        xts[t] = x_t.data + curr_pose
      else:
        xts[t] = x_t.data + xts[t-1]

      # x_plot += list(x_t[:, 0])
      # y_plot += list(x_t[:, 1])
      # if t == 0 or t == 10 or t == 20 or t == 25:
      # plt.scatter(list(x_t[:, 0]), list(x_t[:, 1]), c=colors[t])

      # print "x_t before running_cost", x_t
      # print "mppi: before running_cost max x_t", torch.max(x_t)
      # Calculate costs for each of K trajectories.
      s_time = time.time()
      c = self.running_cost(xts[t], self.goal, self.controls[:, t], py_noise[:, :, t])
      print "MPPI Running cost CPU time: ", time.time() - s_time
      # print "The cost vector for this t is :", c
      # print "mppi: after max x_t", torch.max(x_t)
      # print "x_t after running_cost", x_t

      cost += c.unsqueeze(1)
      # print "c: ", c

    # Debugging Plot of the rollout.
    # plt.scatter(x_plot, y_plot)
    # plt.ylim(-2, 2)
    # plt.xlim(-2, 2)
    # plt.show()

    print "MPPI Time iteration CPU time: ", time.time() - st_time
    min_cost = torch.min(cost)
    print "min_cost:", min_cost

    # Perform the MPPI weighting on your calculatd costs
    # Scale the added noise by the weighting and add to your control sequence
    # Apply the first control values, and shift your control trajectory

    # Definition of eta and weights (immediately after beta in pseudo code)
    normalizer = torch.sum(torch.exp((-1 / self._lambda) * (cost - min_cost)), 0)
    weights = (1 / normalizer) * torch.exp((-1 / self._lambda) * (cost - min_cost))
    weights = weights.squeeze()
    # print "normalizer", normalizer
    # print "weights", weights

    for t in xrange(self.T):
      st_noises = torch.cuda.FloatTensor(noisy_control[t][0])
      vel_noises = torch.cuda.FloatTensor(noisy_control[t][1])
      # print "weights", weights.shape, "st_noises", st_noises.shape, "vel_noise", vel_noises.shape

      st_weighted_noise = weights.dot(st_noises)
      vel_weighted_noise = weights.dot(vel_noises)


      self.controls[1][t] += st_weighted_noise
      self.controls[0][t] += vel_weighted_noise

    # Cap the current controls
    self.controls[1, :][self.controls[1, :] >= MAX_ANGLE] = MAX_ANGLE
    self.controls[1, :][self.controls[1, :] <= -MAX_ANGLE] = -MAX_ANGLE
    self.controls[0, :][self.controls[0, :] >= MAX_VEL] = MAX_VEL
    self.controls[0, :][self.controls[0, :] <= -MAX_VEL] = -MAX_VEL

      # print "st_weighted_noise", st_weighted_noise
      # print "vel_weighted_noise", vel_weighted_noise
    # print "normalizer", normalizer
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

    print("MPPI Compute Time: %4.5f ms" % ((time.time()-t0)*1000.0))
    run_ctrl = self.controls[:, 0]
    new_poses = xts.cpu().numpy() # New poses are in meters
    return run_ctrl, new_poses

  def mppi_cb(self, msg):
    print("-------------------------------------------------callback--------------------------------------------")
    print "x: ", msg.pose.position.x, "y: ", msg.pose.position.y, "theta: ", Utils.quaternion_to_angle(msg.pose.orientation)
    location_in_pix = np.expand_dims(np.array([msg.pose.position.x, msg.pose.position.y, Utils.quaternion_to_angle(msg.pose.orientation)]), axis=0)
    Utils.world_to_map(location_in_pix, self.map_info)
    print "x_pix: ", location_in_pix[:,0], "y_pix: ", location_in_pix[:,1], "theta_pix: ", location_in_pix[:,2]


    if self.last_pose is None:
      print "MPPI: mppi_cb: LAST POSE IS NONE"
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

    pose_dot = curr_pose - self.last_pose # get state
    self.last_pose = curr_pose
    print "setting last pose as curr_pose"

    timenow = msg.header.stamp.to_sec()
    print "timenow:", timenow, "lasttime: ", self.lasttime
    dt = timenow - self.lasttime
    self.lasttime = timenow
    nn_input = torch.cuda.FloatTensor([pose_dot[0], pose_dot[1], pose_dot[2],
                         np.sin(theta),
                         np.cos(theta), 0.0, 0.0, dt])
    s_time = time.time()
    run_ctrl, poses = self.mppi(curr_pose, nn_input)
    print "mppi_cb: mppi call time:", time.time() - s_time

    print "The controls that we are going to run are: ", run_ctrl
    print "The size of the poses returned is ", poses.shape

    self.send_controls(run_ctrl[0], run_ctrl[1])

    self.controls[:, :-1] = self.controls[:, 1:]
    self.controls[:, -1] = 0.0
    # print "controls", self.controls

    print "mppi_cb poses before visualizing"
    v_time = time.time()
    self.visualize(poses)
    print "visualize CPU time", time.time() - v_time

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
    """ Visualizes the poses to RVIZ at `/mppi/paths`
    :param poses: Should be shape (K, T, 3)
    """
    poses_c = poses
    # poses_c = np.reshape(poses_c, (self.K, self.T, 3))
    poses_c = np.transpose(poses_c, axes=(1, 0, 2))
    print "(visualize) self.num_viz_paths", self.num_viz_paths
    print "(visualize) poses_c shape", poses_c.shape
    print "(visualize) poses_c[0]", type(poses_c[0])
    # print "visualizing poses_c", poses_c
    num_connections = self.path_pub.get_num_connections()
    print "(visualize) num_connections:", num_connections
    if num_connections > 0:
      frame_id = 'map'
      for i in range(self.num_viz_paths):
        # print "(visualize) i value: ", i, "poses_c[i,:,:]", poses_c[i,:,:]
        pa = Path()
        pa.header = Utils.make_header(frame_id)
        pa.poses = map(Utils.particle_to_posestamped, poses_c[i,:,:], [frame_id]*self.T)
        self.path_pub.publish(pa)
    else:
      print "(visualize) Not visualizing poses!"

def test_MPPI(mp, N, goal=np.array([0.,0.,0.])):
  print "Testing MPPI"
  init_input = np.array([0.,0.,0.,0.,1.,0.,0.,0.])
  pose = np.array([0.,0.,0.])
  last_pose = np.array([0.,0.,0.])

  mp.goal = goal
  print("Start:", pose)
  mp.ctrl.zero_()
  for i in range(0,N):
    # ROLLOUT your MPPI function to go from a known location to a specified
    # goal pose. Convince yourself that it works.

    # Pose x, y, theta.
    pose_dot = pose - last_pose  # get state
    last_pose = pose

    timenow = time.time()
    lasttime = timenow
    dt = timenow - lasttime
    nn_input = np.array([pose_dot[0], pose_dot[1], pose_dot[2],
                         np.sin(0),
                         np.cos(0), 0.0, 0.0, dt])
    ctrl, pose = mp.mppi(curr_pose=pose, init_input=nn_input)

    print("Now:", pose)
  print("End:", pose)


if __name__ == '__main__':
  print "Entered Main"
  C = 100000
  T = 30
  K = 10
  # (T = 40, K = 2000)
  steering_sigma = 0.15 # These values will need to be tuned
  velocity_sigma = 0.5
  # steering_sigma = 0.0001  # These values will need to be tuned
  # velocity_sigma = 0.0005
  _lambda = 0.01

  # run with ROS
  rospy.init_node("mppi_control", anonymous=True) # Initialize the node
  print "Node initialized;"
  mp = MPPIController(C, T, K, velocity_sigma, steering_sigma, _lambda)
  rospy.spin()

  # test & DEBUG
  # mp = MPPIController(T, K, sigma, _lambda)
  # test_MPPI(mp, 10, np.array([0.,0.,0.]))

