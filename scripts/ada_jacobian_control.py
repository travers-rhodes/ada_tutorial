#!/usr/bin/python
"""
Use gradient descent to move the hand to the cartesian target
"""
import rospy
import numpy as np
import openravepy
import types

import transform_helpers as th
from ada_control_base import AdaControlBase
import ada_cartesian_control

def get_step(nextTransDiff, nextRotDiff):
  rotStepAlpha = 0.1
  transStepAlpha = 1 
  nextDiff = rotStepAlpha * nextRotDiff + transStepAlpha * nextTransDiff
  rospy.logwarn("so, we want to update our joint angles to change by %s" % nextDiff)
  rospy.logwarn("moving in direction %s"%nextDiff)
  return nextDiff
 
class AdaJacobianControl(AdaControlBase):
  def __init__(self, args): 
    super(AdaJacobianControl, self).__init__(args)
    # see also plug_in_cartesian_control in ada_cartesian_control
    self.move_to_target_by_planner = types.MethodType(ada_cartesian_control.move_to_target, self)
    arm_indices = self.manip.GetArmIndices()
    self.robot.SetActiveDOFs(arm_indices)

  # endLoc must be a length 3 np.array
  # if constrainMotion is set to False, don't allow the robot end effector to rotate, and only allow linear motion toward the goal
  # otherwise, don't constrain motion
  def move_to_target(self, endLoc, constrainMotion=False):
    while not self.is_close_enough_to_target(endLoc, epsilon=0.02):
      self.make_step_to_target(endLoc, constrainMotion)

  def should_perform_planned_move(self, curLoc, endLoc):
    return th.distance(curLoc, endLoc) > 0.4

  def make_step_to_target(self, endLoc, constrainMotion):
    with self.env:
      curtrans = self.manip.GetEndEffectorTransform()
      diffTrans, diffRotAxis, diffRotAngle = th.get_transform_difference_axis_angle(curtrans, endLoc, self.quat)
      jac= self.manip.CalculateJacobian()
      angVelJac = self.manip.CalculateAngularVelocityJacobian()
      curLoc = self.manip.GetDOFValues()
      jacobianTranspose = False
      if jacobianTranspose:
        # get the joint change needed in direction of desired translation
        nextTransDiff = np.transpose(jac).dot(diffTrans) 
        # get the joint change needed in direction of desired rotation
        nextRotDiff = th.convert_axis_angle_to_joint(angVelJac, diffRotAxis, diffRotAngle)
        step = get_step(nextTransDiff, nextRotDiff)
      else:
        step = th.least_squares_step(jac, angVelJac, diffTrans, diffRotAxis * diffRotAngle) * 0.1
        desMaxChange = 0.1
        curMaxChange = max(abs(step))
        if curMaxChange > desMaxChange:
          step = step*desMaxChange/curMaxChange
      traj = self.create_two_point_trajectory(curLoc + step)
      if constrainMotion:
        pass
     
    if self.should_perform_planned_move(curtrans[0:3,3], endLoc):
      rospy.logwarn("The target location was too far away, so we're using the slower planner to get there")
      self.move_to_target_by_planner(endLoc) 
    else:
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
