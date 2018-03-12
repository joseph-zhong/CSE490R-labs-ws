#!/usr/bin/env python

import rospy 
import numpy as np
from nav_msgs.srv import GetMap, GetPlan
from lab4.srv import *
from HaltonPlanner import HaltonPlanner
from HaltonEnvironment import HaltonEnvironment
import KinematicModel as model

from geometry_msgs.msg import PoseArray, Pose, PoseStamped

import Utils

PLANNER_SERVICE_TOPIC = '/planner_node/get_plan' # The topic that the planning service is provided on
POST_PROCESS_MAX_TIME = 5.0

class PlannerNode(object):

  def __init__(self):
    # Get the map    
    map_service_name = rospy.get_param("~static_map", "static_map")
    print("Getting map from service: ", map_service_name)
    rospy.wait_for_service(map_service_name)
    self.map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map
        
    graph_file = rospy.get_param("~graph_file", None) # Get the name of the halton graph file
    self.visualize = rospy.get_param("~visualize", False)
    self.environment = HaltonEnvironment(self.map_msg, graph_file, None, None) # Create the Halton Environment
    self.planner = HaltonPlanner(self.environment) #on Create the Halton Planner
    self.server = rospy.Service(PLANNER_SERVICE_TOPIC, GetPlan, self.plan_cb) # Offer planning service
    print 'Ready to plan'
    
  # Called when client wants to generate a plan  
  def plan_cb(self, req):
    # Check that target and source have correct dimension
    if len(req.source) != model.SPACE_DIM or len(req.target) != model.SPACE_DIM:
      return [[],False]
    
    # Check for source == target
    if req.source == req.target:
      result = []
      result.extend(req.source)
      result.extend(req.target)
      return [result, True]
      
    source = np.array(req.source).reshape(3)
    target = np.array(req.target).reshape(3)

    self.environment.set_source_and_target(source, target) # Tell environment what source and target are
    print "HIT"

    # pub = rospy.Publisher("/debug/", PoseArray, queue_size=1)
    # pa = PoseArray()
    # pa.header.frame_id = "map"
    # pa.header.stamp = rospy.Time.now()
    # for i in xrange(self.environment.graph.number_of_nodes()):
    #   config = self.environment.get_config(i)
    #   ps = Pose()
    #   ps.position.x = config[0]
    #   ps.position.y = config[1]
    #   ps.orientation = Utils.angle_to_quaternion(config[2])
    #   pa.poses.append(ps)
    #
    # while not rospy.is_shutdown():
    #   pub.publish(pa)
    #   rospy.sleep(1)



    # print 'hello'
    # pub = rospy.Publisher("/debug/", PoseStamped, queue_size=1)
    # ps = PoseStamped()
    # ps.header.frame_id = "map"
    # ps.header.stamp = rospy.Time.now()
    #
    # ps.pose.position.x = source[0]
    # ps.pose.position.y = source[1]
    # ps.pose.orientation = Utils.angle_to_quaternion(source[2])
    #
    # while not rospy.is_shutdown():
    #   pub.publish(ps)
    #   rospy.sleep(1)




    # Check if planning is trivially infeasible on this environment
    print "Checking if planning is trivially infeasible on this environment..."
    if not self.environment.manager.get_state_validity(source):
      print 'Source in collision'
      return [[],False]

    if not self.environment.manager.get_state_validity(target):
      print 'Target in collision'
      return [[],False]
      
    plan = self.planner.plan() # Find a plan
    
    if plan:
      plan = self.planner.post_process(plan, POST_PROCESS_MAX_TIME) # Try to improve plan
      if self.visualize:
        # Matplotlib.
        print 'hi'
        self.planner.simulate(plan)

        # RViz
        # pub = rospy.Publisher("/debug/", PoseArray, queue_size=1)
        # pa = PoseArray()
        # pa.header.frame_id = "map"
        # pa.header.stamp = rospy.Time.now()
        # for i in xrange(len(plan)):
        #   # config = self.environment.get_config(i)
        #   config = plan[i]
        #   ps = Pose()
        #   ps.position.x = config[0]
        #   ps.position.y = config[1]
        #   ps.orientation = Utils.angle_to_quaternion(config[2])
        #   pa.poses.append(ps)
        #
        # while not rospy.is_shutdown():
        #   pub.publish(pa)
        #   rospy.sleep(1)

      flat_plan = [el for config in plan for el in config] # Convert to a flat list
      return [flat_plan, True]
    else:
      return [[],False]
    
if __name__ == '__main__':
  print 'in main'
  rospy.init_node('planner_node', anonymous=True)
  pn = PlannerNode()
  
  rospy.spin()  
