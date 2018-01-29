#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

if __name__ == '__main__':
  rospy.init_node("path_converter", anonymous=True) # Initialize the node

  my_poses = []
  real_poses = []

  def real_pose_cb(msg):
    real_poses.append(msg)
    real_path = Path()
    real_path.header.stamp = rospy.Time.now()
    real_path.header.frame_id = "map"
    real_path.poses = real_poses
    real_pose_pub.publish(real_path)
    print(msg)

  def my_pose_cb(msg):
    my_poses.append(msg)
    my_path = Path()
    my_path.header.stamp = rospy.Time.now()
    my_path.header.frame_id = "map"
    my_path.poses = my_poses
    my_pose_pub.publish(my_path)
    print("My pose")
    #print(msg)


  real_pose_sub = rospy.Subscriber("/pf/ta/viz/inferred_pose", PoseStamped, real_pose_cb)
  my_pose_sub = rospy.Subscriber("/pf/viz/inferred_pose_throttle", PoseStamped, my_pose_cb)

  real_pose_pub = rospy.Publisher("/pf/real_pose_path", Path, queue_size=1)
  my_pose_pub = rospy.Publisher("/pf/my_pose_path", Path, queue_size=1)



  while not rospy.is_shutdown():
    #print("started")
    pass