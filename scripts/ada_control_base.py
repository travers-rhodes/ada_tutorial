#!/usr/bin/python
"""
This is the base class with some nice template functions for setting up a controller on the ada robot
The move_to_target method should be overwritten in child classes
"""
import rospy
import openravepy
import numpy as np
import time
import adapy
import transforms3d as t3d

import transform_helpers as th

from std_msgs.msg import Header
from geometry_msgs.msg import Point

class AdaControlBase(object):
  def __init__(self, args): 
    openravepy.RaveInitialize(True)
    openravepy.misc.InitOpenRAVELogging()
    self.env, self.robot = adapy.initialize(
        sim=args.sim,
        attach_viewer=args.viewer,
        env_path=args.env_xml
    )
    # make the robot go at half speed
    vel_limits = self.robot.GetDOFVelocityLimits()
    self.robot.SetDOFVelocityLimits(vel_limits * 0.5)
    self.manip = self.robot.SetActiveManipulator("Mico")
    self.manip_rob = openravepy.interfaces.BaseManipulation(self.robot) # create the interface for basic manipulation programs
     
    self.rot = self.generate_target_rotmat()
    
    # even though it's not obvious how we use this, we need to initialize the IKModel on self.robot
    ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(self.robot,iktype=openravepy.IkParameterization.Type.Transform6D)
    try:
      # ideally this would return 0 instead of erroring if it fails, but for now a try-catch will do the trick
      ikmodel.load()
    except:
      ikmodel.autogenerate()

  # generate the rotation matrix corresponding to the desired end-effector rotation
  def generate_target_rotmat(self):
    roty = t3d.quaternions.quat2mat([0,np.sqrt(2), 0, np.sqrt(2)])
    rotx = t3d.quaternions.quat2mat([np.sqrt(2),0, 0, np.sqrt(2)])
    rot = roty.dot(rotx)
    return rot
  
  # return a boolean for whether the end-effector is already at the endLoc target
  # if the end-effector is already close enough to the target, then there's no need
  # to move the end-effector to the target
  def is_close_enough_to_target(self, endLoc):
    return th.distance(self.get_cur_loc(), endLoc) < 0.001

  def get_cur_loc(self):
    Tee = self.manip.GetEndEffectorTransform()
    curLoc = Tee[0:3,3]
    return(curLoc)

  # for at most timeoutSecs, compute and move toward the input endLoc
  # endLoc must be a length 3 np.array
  # if constrainMotion is set to False, don't allow the robot end effector to rotate, and only allow linear motion toward the goal
  # otherwise, don't constrain motion
  def move_to_target(self, endLoc, timeoutSecs=0, constrainMotion=False):
    '''
    This move_to_target method should be overwritten in child classes
    '''
    raise NotImplementedError("This move_to_target method should be overwritten in child classes")
