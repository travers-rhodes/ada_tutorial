#!/usr/bin/python
"""Shows how to set a physics engine and send torque commands to the robot
"""
import rospy
import numpy as np
import time
import adapy
import argparse
import tf
from enum import Enum

from ada_tutorial.srv import MoveArm
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped

point_topic = "/DO/inferenceOut/Point"
point_stamped_topic = point_topic + "Stamped"

class CameraCalibration:
  def __init__(self):
    camera_to_camera = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    self.camera_to_robot =  np.array([[1,0,0,0.4],[0,0,1,-0.8],[0,-1,0,0.30],[0,0,0,1]])
    """
    Copied from output from Alexandre's calibration. TODO put this in a parameter server
    'Rotation is: ', array([[ 0.99788558,  0.01727208, -0.06265819],
          [ 0.06467983, -0.16904555,  0.98348367],
          [ 0.00639472, -0.98545689, -0.16980528]]))
    ('Translation is:', array([[ 0.44780474],
          [-0.95989491],
          [ 0.43955628]]))
    """
    self.camera_to_robot = np.array([[0.99788558,  0.01727208, -0.06265819, 0.44780474],
                                     [ 0.06467983, -0.16904555,  0.98348367, -0.95989491],
                                     [ 0.00639472, -0.98545689, -0.16980528, 0.43955628],
                                     [ 0, 0, 0, 1]])
    self.br = tf.TransformBroadcaster()
    rospy.logwarn("camera calibration initialized")
    
    rospy.Timer(rospy.Duration(0.01), self.broadcastTransform)
    rospy.logwarn("sent first message")

  def broadcastTransform(self, event):
    #rospy.logwarn("broadcasting tf")
    self.br.sendTransform(self.camera_to_robot[0:3,3],
                          # note we pass in 4x4 to this method... https://github.com/ros/geometry/issues/64
                          tf.transformations.quaternion_from_matrix(self.camera_to_robot),
                          rospy.Time.now(),
                          "camera_rgb_optical_frame",
                          "world")

  # point should be a length 4 np.array giving the location of the target point in the camera frame
  def convert_to_robot_frame(self, point):
    return self.camera_to_robot.dot(point.transpose())

class Tracker:
  def __init__(self):
    self.cameraCalib = CameraCalibration()
    rospy.wait_for_service('update_track_target', timeout=None)
    self.pubStamped = rospy.Publisher(point_stamped_topic, PointStamped, queue_size=10)
    self._update_target = rospy.ServiceProxy('update_track_target', MoveArm)
    self.target_listener = None

  #### PUBLIC METHODS
  # we try to make it so that all of these methods are idempotent 
  # and can be called from any state
  def start_updating_target_relative_to_mouth(self, robot_coord_offset=[0,0,0]):
    self._stop_updating_target() 
    self.target_listener = rospy.Subscriber(point_topic, Point, self._update_target_camera_frame, (robot_coord_offset))
  
  def start_tracking_fixed_target(self, robot_coord_point):
    self._stop_updating_target() 
    # you just send the target point, you don't need to continually update it
    self._update_target(target=Point(robot_coord_point[0], robot_coord_point[1], robot_coord_point[2]), constrainMotion=True)

  def stop_moving(self):
    self._stop_updating_target()
    self._update_target(target=None, constrainMotion=False)


  #### PRIVATE METHODS #####
  def _stop_updating_target(self):
    if (self.target_listener is not None):
      self.target_listener.unregister()

  # return an np.array of the [x,y,z] target mouth location
  # in the coordinate frame of the robot base
  def _convert_camera_to_robot_frame(self, mouth_pos):
    t = mouth_pos
    point_in_camera_frame = np.array([t.x, t.y, t.z, 1])
    point_in_robot_frame = self.cameraCalib.convert_to_robot_frame(point_in_camera_frame)
    return point_in_robot_frame[0:3]

  # compute and move toward the target mouth location
  # only move for at most timeoutSecs,   
  # if constrainMotion is set to False, don't allow the robot end effector to rotate, and only allow linear motion toward the goal
  # otherwise, don't constrain motion
  def _update_target_camera_frame(self, mouth_pos, robot_coord_offset = [0,0,0]):
    endLoc = self._convert_camera_to_robot_frame(mouth_pos) + np.array(robot_coord_offset)
    h = Header()
    h.stamp = rospy.Time.now()
    h.frame_id = "camera_rgb_optical_frame"
    self.pubStamped.publish(PointStamped(h,mouth_pos))
    self._update_target(target=Point(endLoc[0], endLoc[1], endLoc[2]), constrainMotion=True)
