#!/usr/bin/python
"""Shows how to set a physics engine and send torque commands to the robot
"""
import rospy
import argparse
from openravepy import *
import numpy as np
import time
import adapy
import transforms3d as t3d
import threading

from std_msgs.msg import Header
from tf.msg import tfMessage
from geometry_msgs.msg import TransformStamped, Transform, Vector3


class PositionCache:
  def __init__(self):
    self.target_listener = rospy.Subscriber("/feedbot/mouth_pose", tfMessage, self.target_listener_callback)
    # save off the latest mouth pose tf object seen by the vision system in a cached value
    # this object has
    # * Header header (int seq; time stamp; string frame_id)
    # * string child_frame_id
    # * geometry_msgs/Transform transform (Vectory3 translation; Quaternion rotation)
    self.latest_mouth_transform = None

  def target_listener_callback(self, mesg):
    self.latest_mouth_transform = mesg.transforms[0]

class Tracker:
  def __init__(self, args):
    RaveInitialize(True)
    misc.InitOpenRAVELogging()
    
    self.env, self.robot = adapy.initialize(
        sim=args.sim,
        attach_viewer=args.viewer,
        env_path=args.env_xml
    )
    
    self.manip = self.robot.SetActiveManipulator("Mico")
    self.manip_rob = interfaces.BaseManipulation(self.robot) # create the interface for basic manipulation programs

    # even though it's not obvious how we use this, we need to initialize the IKModel
    ikmodel = databases.inversekinematics.InverseKinematicsModel(self.robot,iktype=IkParameterization.Type.Transform6D)
    if not ikmodel.load():
      ikmodel.autogenerate()

    self.rot = self.generate_target_rotmat()

    self.pos_cache = PositionCache()

    # move to init position 
    self.move_to_target(np.array([ 0.21676946, -0.2959789,   0.552686472]), timeoutSecs=0, allowRotation=True)

  def follow_mouth(self,timeoutSecs):
    # start tracking the actual mouth
    while not rospy.is_shutdown():
      self.track_target(timeoutSecs)
    

  # return an np.array of the [x,y,z] target mouth location
  # in the coordinate frame of the robot base
  def get_target_loc(self):
    if self.pos_cache.latest_mouth_transform is None:
      return None
    t =  self.pos_cache.latest_mouth_transform.transform.translation
    return np.array([t.x, t.y, t.z])

  # generate the rotation matrix corresponding to the desired end-effector rotation
  def generate_target_rotmat(self):
    roty = t3d.quaternions.quat2mat([0,np.sqrt(2), 0, np.sqrt(2)])
    rotx = t3d.quaternions.quat2mat([np.sqrt(2),0, 0, np.sqrt(2)])
    rot = roty.dot(rotx)
    return rot

  # compute and move toward the target mouth location
  # only move for at most timeoutSecs,   
  # if allowRotation is set to True, allow the robot end effector to rotate along the x,y axes. 
  # Otherwise, only allow rotation around the z-axis
  def track_target(self, timeoutSecs, allowRotation=False):
    endLoc = self.get_target_loc() 
    if endLoc is None:
      rospy.sleep(timeoutSecs)
      return
    self.move_to_target(endLoc, timeoutSecs, allowRotation=False)

  # return a boolean for whether the end-effector is already at the endLoc target
  # if the end-effector is already close enough to the target, then there's no need
  # to move the end-effector to the target
  def is_close_enough_to_target(self, endLoc):
    return self.distance_to_target(endLoc) < 0.001

  def distance_to_target(self, endLoc):
    curLoc = self.get_cur_loc()
    diff = endLoc - curLoc
    dist = np.sqrt(sum(diff**2))
    print(dist)
    return dist

  def get_cur_loc(self):
    Tee = self.manip.GetEndEffectorTransform()
    curLoc = Tee[0:3,3]
    return(curLoc)

  # for at most timeoutSecs, compute and move toward the input endLoc
  # endLoc must be a length 3 np.array
  # if allowRotation is set to True, allow the robot end effector to rotate along the x,y axes. 
  # Otherwise, only allow rotation around the z-axis
  def move_to_target(self, endLoc, timeoutSecs, allowRotation=False):
    if self.is_close_enough_to_target(endLoc):
      rospy.sleep(timeoutSecs)
      return 
    with self.env:
      print("cur loc", self.get_cur_loc())
      print("moving to", endLoc)
      Tgoal = np.concatenate((np.concatenate((self.rot, np.transpose([endLoc])), axis=1),[[0,0,0,1]]), axis=0)
      print("moving to", Tgoal)
      constrainttaskmatrix=np.eye(4)
      constraintmatrix = np.linalg.inv(self.manip.GetTransform())
      constrainterrorthresh = 0.005 # copied from tutorial
      constraintfreedoms = [0,0,0,0,0,0] # based on parsing the tutorial. Not sure why it's not 0,0,1,...
      if not allowRotation:
       # don't let any x,y rotations happen
       constraintfreedoms = [1,1,0,0,0,0] 
      self.manip_rob.MoveToHandPosition(matrices=[Tgoal],maxiter=3000,maxtries=10,seedik=40,
        constraintfreedoms=constraintfreedoms,
        constraintmatrix=constraintmatrix, 
        constrainttaskmatrix=constrainttaskmatrix,
        constrainterrorthresh=constrainterrorthresh,
        steplength=0.001)
    self.robot.WaitForController(timeoutSecs) # wait the timeout amount before choosing a new target

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
  thread = Thread(target=t.follow_mouth, args = (0,))
  thread.start()
  poses =[ 
                [ 0.21676946, -0.2959789,   0.552686472],
                [ 0.21676946, -0.2959789,   0.52686472],
                [ 0.21676946, -0.2959789,   0.42686472],
                [ 0.31676946, -0.1959789,   0.42686472],
                [ 0.322638  , -0.35740914,  0.43557774],
                [ 0.31676946, -0.3959789,   0.42686472]]
  pub = rospy.Publisher('feedbot/mouth_pose', tfMessage, queue_size=10)
  h = Header()
  h.stamp = rospy.Time.now()
  for pose in poses:
    mesg = tfMessage([TransformStamped(h, "dummy_frame", Transform(Vector3(pose[0], pose[1], pose[2]), None))])
    pub.publish(mesg)
    rospy.sleep(1)
  thread.join()


