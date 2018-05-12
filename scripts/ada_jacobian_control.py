#!/usr/bin/python
"""
Use gradient descent to move the hand to the cartesian target
"""
import rospy
import numpy as np
import openravepy

import transform_helpers as th
from ada_control_base import AdaControlBase

class AdaJacobianControl(AdaControlBase):
  def __init__(self, args): 
    super(AdaJacobianControl, self).__init__(args)

  # endLoc must be a length 3 np.array
  # if constrainMotion is set to False, don't allow the robot end effector to rotate, and only allow linear motion toward the goal
  # otherwise, don't constrain motion
  def move_to_target(self, endLoc, constrainMotion=False):
    arm_indices = self.manip.GetArmIndices()
    self.robot.SetActiveDOFs(arm_indices)
    while not self.is_close_enough_to_target(endLoc):
      self.make_step_to_target(endLoc, constrainMotion)

  def make_step_to_target(self, endLoc, constrainMotion):
    with self.env:
      curtrans = self.manip.GetEndEffectorTransform()
      #rospy.logwarn("curtrans is %s" % curtrans)
      dir_to_go = th.get_transform_difference(curtrans, endLoc, self.quat)
      #rospy.logwarn("dir_to_go is %s"%dir_to_go)
      jac = self.manip.CalculateJacobian()
      #rospy.logwarn("jacobian is %s"%jac)
      curLoc = self.manip.GetDOFValues()
      #rospy.logwarn("curLoc is %s"%curLoc)
      nextLoc = np.transpose(jac).dot(dir_to_go[0:3]) 
      #rospy.logwarn("nextLoc is %s"%nextLoc)
      nextLoc = curLoc + nextLoc
      traj = self.create_two_point_trajectory(nextLoc)
      if constrainMotion:
        pass
      
    #rospy.logwarn(traj) 
    self.robot.ExecuteTrajectory(traj)

  def create_two_point_trajectory(self,goal_joint_values):
    # manipulator has 6DOF, but we also need to specify the finger DOFs, apparently
    # so we get the full DOFs here by concatenating on two extra DOFs
    extraDOFValues = []
    curLoc = self.manip.GetDOFValues()
    #rospy.logwarn("curFullLoc is %s"%curLoc)
    traj = openravepy.RaveCreateTrajectory(self.env,'')
    traj.Init(self.robot.GetActiveConfigurationSpecification())
    traj.Insert(0,np.concatenate((curLoc, extraDOFValues)))
    traj.Insert(1,np.concatenate((goal_joint_values, extraDOFValues)))
    openravepy.planningutils.RetimeActiveDOFTrajectory(traj,self.robot)#,hastimestamps=False,maxvelmult=1,plannername='ParabolicTrajectoryRetimer')

    return traj
