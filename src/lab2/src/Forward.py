#!/usr/bin/env python


import cv2
from cv_bridge import CvBridge, CvBridgeError
import math
import matplotlib.pyplot as plt
import numpy as np
from pprint import pprint
import sys


from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped
import tf
from tf import transformations
import rospy

from convolve import convolve
from util import _mask_img


# TODO: Change to RED
boundaries = [
  ((05, 100, 100), (35, 240, 220))
]

SLEEP_TIME = 5.0


CAMERA_ANGLE = 0.0
CAMERA_FRAME_PARENT = 'camera_rgb_optical_frame'
CAMERA_FRAME_CHILD = 'base_link'



### For template creation
MAX_ANGLE = 0.20
NUM_TEMPLATES = 29
NUM_PTS = 400
V = 0.46  # Car's current velocity
CAR_LEN = 0.33


def create_template(steering):
    """ Uses the Kinematic Model to create a template using 'steering'
        as a constant steering angle."""

    init_pt = (0, 0)

    X = []
    Y = []

    last_pos = (0, 0)
    theta = 0
    for i in range(NUM_PTS):
        dt = 0.02
        delta_theta = V / CAR_LEN * np.sin(steering) * dt

        # Car should go straight if the steering angle is 0.
        if steering == 0.0:
            delta_x = V * dt
            delta_y = 0
        else:
            delta_x = CAR_LEN / np.sin(steering) * (np.sin(theta + delta_theta) - np.sin(theta))
            delta_y = CAR_LEN / np.sin(steering) * (np.cos(theta) - np.cos(theta + delta_theta))

        x = delta_x + last_pos[0]
        y = delta_y + last_pos[1]

        X.append(x)
        Y.append(y)

        theta += delta_theta
        last_pos = (x, y)

    return X, Y


class ForwardController(object):

  def __init__(self, control_pub, image_pub):
    self.cvBridge = CvBridge()
    self.tl = tf.TransformListener()
    self.tb = tf.TransformBroadcaster()
    rospy.sleep(rospy.Duration(SLEEP_TIME))
    self.tl.waitForTransform(CAMERA_FRAME_PARENT, CAMERA_FRAME_CHILD, rospy.Time(), rospy.Duration(5.0))
    translation, rotation = self.tl.lookupTransform(CAMERA_FRAME_PARENT, CAMERA_FRAME_CHILD, rospy.Time())
    x, y, z = transformations.euler_from_quaternion(rotation)
    rot_matrix = transformations.euler_matrix(x + CAMERA_ANGLE, y, z)  # I am not positive about this

    translation = list(translation) + [1]
    rot_matrix[:, -1] = translation
    rot_matrix = np.array(rot_matrix)
    print(rot_matrix)

    # tranform = TransformStamped()
    # tranform.transform.rotation.x = rotation[0]
    # tranform.transform.rotation.y = rotation[1]
    # tranform.transform.rotation.z = rotation[2]
    # tranform.transform.rotation.w = rotation[3]
    # tranform.transform.translation.x = translation[0]
    # tranform.transform.translation.y = translation[1]
    # tranform.transform.translation.z = translation[2]
    # tranform.header.frame_id = 'base_link'
    # tranform.child_frame_id = 'my_frame'
    # new_rotation = transformations.euler_from_quaternion(rotation)
    # new_rotation[0] += math.pi
    # new_rotation = tuple(new_rotation)
    # x, y, z = new_rotation
    # x += 0.1
    # translation[0] += 0
    # new_rotation = transformations.quaternion_from_euler(x, y, z)
    # pprint((x, y, z))
    # pprint(rotation)

    #self.tl.setTransform(tranform)
    self.control_pub = control_pub
    self.image_pub = image_pub

    templates = []
    self.robot_frames = []
    self.camera_frames = []
    self.pixel_frames = None

    # Discretize angles, one per template.
    self.discretized_angles = np.linspace(-MAX_ANGLE, MAX_ANGLE, NUM_TEMPLATES)
    for theta in self.discretized_angles:

        print theta
        Xw, Yw = create_template(theta)
        Zw = np.zeros(NUM_PTS)  # W.r.t car's frame (base link)
        ones = np.ones(NUM_PTS)  # Addition of 1 allows rotation multiplication

        # Convert from robot_frame to camera frame
        robot_frame = np.array([Xw, Yw, Zw, ones])
        camera_frame = rot_matrix.dot(robot_frame)
        self.robot_frames.append(robot_frame)
        self.camera_frames.append(camera_frame)

    # Plot Robot Frames
#    Xs, Ys = np.array([]), np.array([])
#    for rf in self.robot_frames:
#       Xs = np.append(Xs, rf[0])
#       Ys = np.append(Ys, rf[1])

#    plt.scatter(Xs, Ys)
#    plt.show()

    # Plot Camera Frames
#    Xc, Yc = np.array([]), np.array([])
#    for cf in self.camera_frames:
#       print cf
#       Xc = np.append(Xc, cf[0])
#       Yc = np.append(Yc, cf[2])


#    plt.scatter(Xc, Yc)
#    axes = plt.gca()
#    axes.set_ylim([0, 2.0])   

#    plt.show()


  def image_cb(self, msg):
    brg_img = self.cvBridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    hsv_img = cv2.cvtColor(brg_img, cv2.COLOR_BGR2HSV)
    mask_img = _mask_img(hsv_img, boundaries)

    self.visualize(image=mask_img)
    mask_img = np.rollaxis(mask_img, 2, 0)

    if self.pixel_frames is not None:
      score_templates = []
      print "# OF TEMPLATES:", len(self.pixel_frames)
      for pxf in self.pixel_frames:
        score = np.sum(convolve(pxf, mask_img))
        score_templates.append(score)

      print "# OF SCORES", len(score_templates)
      best_template_idx = np.argmax(score_templates)

      print "SCORES", score_templates

      # Templates align with the angles that created them.
      predicted_control = self.discretized_angles[best_template_idx]
      print "CONTROL: ", predicted_control, "INDEX", best_template_idx
 
      self.publish_controls(predicted_control)


  def publish_controls(self, steering_angle):
    ads = AckermannDriveStamped()
    ads.drive.steering_angle = steering_angle
    ads.drive.speed = V
    self.control_pub.publish(ads)


  def visualize(self, steering_angle=0, image=0):
      print "Visualizing Image:"
      rosImg = self.cvBridge.cv2_to_imgmsg(image)
      self.image_pub.publish(rosImg)



  def k_cb(self, msg):
    if self.pixel_frames is None:
        k = msg.K
        self.pixel_frames = []
        K = np.array(list(k)).reshape(3, 3)

        # Instantiate pixel frames from camera frames
        for camera_frame in self.camera_frames:
            x_prime = camera_frame[0]
            y_prime = camera_frame[1]
            z_prime = camera_frame[2]

            # Instill camera intrinsics and convert
            # to pixel frame
            u = x_prime / z_prime
            v = y_prime / z_prime
            ones = np.ones(NUM_PTS)
            pixel_frame = np.array([u, v, ones])
            pixel_frame = K.dot(pixel_frame)

            # Invert pixel frames and crop to match perspective.
            pixel_image = np.zeros((480, 640))
            for column in pixel_frame.T:
                # Plot the point if within image dimensions
                u, v = column[0], column[1]
                if u >= 0.0 and u <= 480 and v >= 0.0 and v <= 640:
                    u = np.floor(u)
                    v = np.floor(v)
                    pixel_image[u][v] = 1

            # plt.imshow(pixel_image, cmap='Greys')
            # plt.show()
            self.pixel_frames.append(pixel_image)

        self.pixel_frames = np.array(self.pixel_frames)
#        Us = np.array([])
#        Vs = np.array([])
#        for pixel_frame in self.pixel_frames:
#           u = pixel_frame[0]
#            v = pixel_frame[1]
#            Us = np.append(Us, u)
#            Vs = np.append(Vs, v)

#        plt.scatter(Us, Vs)
#        axes = plt.gca()
#        axes.set_ylim([0, 480])
#        axes.set_xlim([0, 640])
#        axes.invert_yaxis()

#        plt.show()



