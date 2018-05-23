#!/usr/bin/python
"""Shows how to set a physics engine and send torque commands to the robot
"""
import rospy
import numpy as np
import time
import adapy
import argparse
import tf
import yaml
import rospkg

from ada_tutorial.srv import TrackPose
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion 


class CameraCalibration:
  def __init__(self):
    """
    cameraCalibration.yml is something like:
    Rotation:
    - - 0.9989181262157819
      - 0.01931543906095123
      - -0.04230237500938314
    - - 0.04514525926005835
      - -0.18453994506190463
      - 0.981787611575381
    - - 0.011157180821961618
      - -0.9826351929838623
      - -0.18521229663733005
    Translation:
    - - 0.5391208676662875
    - - -0.6604205518057711
    - - 0.4549267234161455
    """
    rospack = rospkg.RosPack()
    ada_tut_path = rospack.get_path("ada_tutorial")
    with open(ada_tut_path + '/config/calibrationParameters.yml','r') as f:
      calibParams = yaml.load(f)
      rospy.logwarn("calibParams %s"% calibParams)
      rotation = np.array(calibParams["Rotation"])
      translation = np.array(calibParams["Translation"])
      
      self.camera_to_robot = np.concatenate((np.concatenate((rotation,translation), axis=1),
                                     [[ 0, 0, 0, 1]]))
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

class TrackerInterface:
  def __init__(self, defaultQuat):
    self.cameraCalib = CameraCalibration()
    rospy.wait_for_service('update_pose_target', timeout=None)
    self._update_target = rospy.ServiceProxy('update_pose_target', TrackPose)
    self.pose_target_listener = None
    self.mouth_target_listener = None
    self.defaultQuat = defaultQuat 

  #### PUBLIC METHODS
  # we try to make it so that all of these methods are idempotent 
  # and can be called from any state
  def start_updating_target_to_pose(self, target_pose_topic, robot_coord_offset=[0,0,0]):
    self._stop_updating_target() 
    self.pose_target_listener = rospy.Subscriber(target_pose_topic, Pose, self._update_target_pose_robot_frame, (robot_coord_offset))

  def start_updating_target_to_point(self, mouth_point_topic, robot_coord_offset=[0,0,0]):
    self._stop_updating_target() 
    self.mouth_target_listener = rospy.Subscriber(mouth_point_topic, Point, self._update_target_camera_frame, (robot_coord_offset))
  
  def start_tracking_fixed_target(self, robot_coord_point):
    self._stop_updating_target() 
    # you just send the target point, you don't need to continually update it
    self._update_target(target=Pose(Point(robot_coord_point[0], robot_coord_point[1], robot_coord_point[2]), self.defaultQuat))

  def stop_moving(self):
    self._stop_updating_target()
    self._update_target(stopMotion=True)


  #### PRIVATE METHODS #####
  def _stop_updating_target(self):
    if (self.mouth_target_listener is not None):
      self.mouth_target_listener.unregister()
    if (self.pose_target_listener is not None):
      self.pose_target_listener.unregister()

  # return an np.array of the [x,y,z] target mouth location
  # in the coordinate frame of the robot base
  def _convert_camera_to_robot_frame(self, mouth_pos):
    t = mouth_pos
    point_in_camera_frame = np.array([t.x, t.y, t.z, 1])
    point_in_robot_frame = self.cameraCalib.convert_to_robot_frame(point_in_camera_frame)
    return point_in_robot_frame[0:3]

  # compute and move toward the target mouth location
  # only move for at most timeoutSecs,   
  def _update_target_camera_frame(self, mouth_pos, robot_coord_offset = [0,0,0]):
    endLoc = self._convert_camera_to_robot_frame(mouth_pos) + np.array(robot_coord_offset)
    self._update_target(target=Pose(Point(endLoc[0], endLoc[1], endLoc[2]), self.defaultQuat))
  
  # move toward the target spoon pose
  def _update_target_pose_robot_frame(self, target_pose, robot_coord_offset = [0,0,0]):
    newPosition = Point(target_pose.position.x + robot_coord_offset[0],
                        target_pose.position.y + robot_coord_offset[1],
                        target_pose.position.z + robot_coord_offset[2])
    self._update_target(target=Pose(newPosition, target_pose.orientation))



