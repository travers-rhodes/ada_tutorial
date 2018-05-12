#!/usr/bin/python
"""
Use gradient descent to move the hand to the cartesian target
"""
import rospy
import numpy as np
import openravepy

import transform_helpers as th
from ada_control_base import AdaControlBase

def get_next_loc(curLoc, nextTransDiff, nextRotDiff):
  nextRotDiff = np.multiply(np.array([0,0,0,0,0,1]), nextRotDiff) * 0.1
  nextRotDiff = np.array([0,0,0,0,0,1]) * 0.1
  nextDiff = -nextRotDiff# + nextTransDiff
  curMaxChange = max(abs(nextDiff))
  #if curMaxChange > desMaxChange:
  #  nextDiff = nextDiff*desMaxChange/curMaxChange
  rospy.logwarn("so, we want to update our joint angles to change by %s" % nextDiff)
  rospy.logwarn("moving in direction %s"%nextDiff)
  return curLoc + nextDiff

def multiply_rotation_jacobian_with_quat(rotjac, dir_to_go):
  # t3d wants the scalar part to come first
  t3dRotJacobian = np.concatenate((rotjac[3:4,:], rotjac[0:3,:]))
  return th.mult_jacobian_with_direction(t3dRotJacobian, dir_to_go[3:7])

class AdaJacobianControl(AdaControlBase):
  def __init__(self, args): 
    super(AdaJacobianControl, self).__init__(args)

  # endLoc must be a length 3 np.array
  # if constrainMotion is set to False, don't allow the robot end effector to rotate, and only allow linear motion toward the goal
  # otherwise, don't constrain motion
  def move_to_target(self, endLoc, constrainMotion=False):
    arm_indices = self.manip.GetArmIndices()
    self.robot.SetActiveDOFs(arm_indices)
    while not self.is_close_enough_to_target(endLoc, epsilon=0.01):
      self.make_step_to_target(endLoc, constrainMotion)

  def make_step_to_target(self, endLoc, constrainMotion):
    with self.env:
      curtrans = self.manip.GetEndEffectorTransform()
      rospy.logwarn("Destination quat is %s" % self.quat)
      dir_to_go = th.get_transform_difference(curtrans, endLoc, self.quat)
      rospy.logwarn("this silly computer is saying we need to go %s"%dir_to_go[3:7])
      jac = self.manip.CalculateJacobian()
      rotjac = self.manip.CalculateRotationJacobian()
      rospy.logwarn("rotjac is %s"%rotjac)
      curLoc = self.manip.GetDOFValues()
      nextTransDiff = np.transpose(jac).dot(dir_to_go[0:3]) 
      nextRotDiff = multiply_rotation_jacobian_with_quat(rotjac, dir_to_go)
      nextLoc = get_next_loc(curLoc, nextTransDiff, nextRotDiff)
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
