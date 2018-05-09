#!/usr/bin/python
"""Shows how to set a physics engine and send torque commands to the robot
"""
import rospy
import argparse
import numpy as np
import time
import adapy

from ada_cartesian_control import AdaCartesianControl

from std_msgs.msg import Header
from geometry_msgs.msg import Point

point_topic = "/DO/inferenceOut/Point"
x_dist = 0.41676946

class CameraCalibration:
  def __init__(self):
    camera_to_camera = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    self.camera_to_robot =  np.array([[0,0,1,-0.8],[-1,0,0,-0.4],[0,-1,0,0.55],[0,0,0,1]]).dot(camera_to_camera)
    self.camera_to_robot = np.eye(4)

  # point should be a length 4 np.array giving the location of the target point in the camera frame
  def convert_to_robot_frame(self, point):
    return self.camera_to_robot.dot(point.transpose())

class PositionCache:
  def __init__(self):
    self.target_listener = rospy.Subscriber(point_topic, Point, self.target_listener_callback)
    # save off the latest mouth pose tf object seen by the vision system in a cached value
    # this object has
    # * Header header (int seq; time stamp; string frame_id)
    # * string child_frame_id
    # * geometry_msgs/Transform transform (Vectory3 translation; Quaternion rotation)
    self.latest_mouth_position = None

  def target_listener_callback(self, mesg):
    self.latest_mouth_position = mesg

class Tracker:
  def __init__(self, args):
    self.cart_con = AdaCartesianControl(args)
    self.pos_cache = PositionCache()
    self.cameraCalib = CameraCalibration()

    # move to init position 
    self.cart_con.move_to_target(np.array([x_dist, -0.2959789,   0.552686472]), timeoutSecs=0, constrainMotion=False)

  def follow_mouth(self,timeoutSecs):
    # start tracking the actual mouth
    while not rospy.is_shutdown():
      self.track_target(timeoutSecs)

  # return an np.array of the [x,y,z] target mouth location
  # in the coordinate frame of the robot base
  def get_target_loc(self):
    if self.pos_cache.latest_mouth_position is None:
      return None
    t = self.pos_cache.latest_mouth_position
    point_in_camera_frame = np.array([t.x, t.y, t.z, 1])
    point_in_robot_frame = self.cameraCalib.convert_to_robot_frame(point_in_camera_frame)
    print(point_in_robot_frame)
    return point_in_robot_frame[0:3]

  # compute and move toward the target mouth location
  # only move for at most timeoutSecs,   
  # if constrainMotion is set to False, don't allow the robot end effector to rotate, and only allow linear motion toward the goal
  # otherwise, don't constrain motion
  def track_target(self, timeoutSecs, constrainMotion=True):
    endLoc = self.get_target_loc() 
    if endLoc is None:
      if timeoutSecs == 0:
        rospy.sleep(1)
      else:
        rospy.sleep(timeoutSecs)
      return
    self.cart_con.move_to_target(endLoc, timeoutSecs, constrainMotion=True)

if __name__=="__main__":
  # parse input arguments
  parser = argparse.ArgumentParser(description='node to track the position given by a rostopic')
  parser.add_argument('-s', '--sim', action='store_true',
                          help='simulation mode')
  parser.add_argument('-v', '--viewer', nargs='?', const=True,
                          help='attach a viewer of the specified type')
  parser.add_argument('--env-xml', type=str,
                          help='environment XML file; defaults to an empty environment')
  parser.add_argument('--debug', action='store_true',
                          help='enable debug logging')
  args = parser.parse_args()
 
  # initialize the ros node 
  rospy.init_node('adapy_tracker', anonymous=True)
  # start tracker running and start following locations
  t = Tracker(args)
  from threading import Thread
  thread = Thread(target=t.follow_mouth, args = (1,))
  thread.start()

  rospy.spin()

  pub = rospy.Publisher(point_topic, Point, queue_size=10)
  h = Header()
  h.stamp = rospy.Time.now()
  times = np.array(range(100))
  y_center = -0.195
  z_center = 0.45
  poses = [[ x_dist, y_center + 0.1 * np.sin(t),z_center + 0.1 * np.cos(t)] for t in times]

  for pose in poses+poses+poses:
    #mesg = tfMessage([TransformStamped(h, "dummy_frame", Transform(Vector3(pose[0], pose[1], pose[2]), None))])
    mesg = Point(pose[0], pose[1], pose[2])
    pub.publish(mesg)
    rospy.sleep(2)
  thread.join()


