#!/usr/bin/python
"""
Use gradient descent to move the hand to the cartesian target
"""
import rospy
import numpy as np
import openravepy
import types

from std_msgs.msg import Float64

import transform_helpers as th
from ada_control_base import AdaControlBase
import ada_cartesian_control

def get_step(nextTransDiff, nextRotDiff):
  rotStepAlpha = 0.1
  transStepAlpha = 1 
  nextDiff = rotStepAlpha * nextRotDiff + transStepAlpha * nextTransDiff
  #rospy.logwarn("so, we want to update our joint angles to change by %s" % nextDiff)
  #rospy.logwarn("moving in direction %s"%nextDiff)
  return nextDiff
 
class AdaJacobianControl(AdaControlBase):
  def __init__(self, args, endEffName="Spoon"): 
    super(AdaJacobianControl, self).__init__(args, endEffName)
    # see also plug_in_cartesian_control in ada_cartesian_control
    self.move_to_target_by_planner = types.MethodType(ada_cartesian_control.move_to_target, self)
    self.arm_indices = self.manip.GetArmIndices()
    self.robot.SetActiveDOFs(self.arm_indices)
    self.joint_min, self.joint_max = self.robot.GetDOFLimits(self.arm_indices)

  # endLoc must be a length 3 np.array
  # if constrainMotion is set to False, don't allow the robot end effector to rotate, and only allow linear motion toward the goal
  # otherwise, don't constrain motion
  def move_to_target(self, endLoc, constrainMotion=False):
    while not self.is_close_enough_to_target(endLoc, epsilon=0.02):
      self.make_step_to_target(endLoc, constrainMotion)

  def should_perform_planned_move(self, curLoc, endLoc):
    return th.distance(curLoc, endLoc) > 1
  
  def get_pseudo_endLoc(self, curCartLoc, endLoc):
    rospy.logwarn("CURRENT POSITION is %s" % curCartLoc)
    breaks = [-0.15, -0.07, 0, 0.07]
    pseudoEndLoc = np.array(endLoc)
    for i in range(3):
      if curCartLoc[1] < breaks[i] and endLoc[1] > breaks[i+1]:
        pseudoEndLoc[1] = (breaks[i+1] + breaks[i])/2.0
        if endLoc[1] - curCartLoc[1] > 0.15:
          pseudoEndLoc[0] = endLoc[0] + 0.1
        return pseudoEndLoc
      if curCartLoc[1] > breaks[i+1] and endLoc[1] < breaks[i]:
        pseudoEndLoc[1] = (breaks[i+1] + breaks[i])/2.0
        pseudoEndLoc[0] = endLoc[0] + 0.1
        return pseudoEndLoc
    rospy.logwarn("pseudoEndLoc is %s"% pseudoEndLoc)
    return pseudoEndLoc

  def make_step_to_target(self, endLoc, constrainMotion):
    stepSize = 0.02
    # don't do anything if close enough to target
    if self.is_close_enough_to_target(endLoc, epsilon=stepSize):
      return
    curtrans = self.manip.GetEndEffectorTransform()
    curCartLoc = curtrans[0:3,3]
    pseudoEndLoc = self.get_pseudo_endLoc(curCartLoc, endLoc)
    self.make_step_to_pseudotarget(pseudoEndLoc, constrainMotion, stepSize)
    curtrans = self.manip.GetEndEffectorTransform()
    curCartLoc = curtrans[0:3,3]
    delta = th.distance(curCartLoc, endLoc)
    self.dist_to_goal_publisher.publish(Float64(delta))
 
  def make_step_to_pseudotarget(self, endLoc, constrainMotion, stepSize):
    with self.env:
      curtrans = self.manip.GetEndEffectorTransform()
      diffTrans, diffRotAxis, diffRotAngle = th.get_transform_difference_axis_angle(curtrans, endLoc, self.quat)
      jac= self.manip.CalculateJacobian()
      angVelJac = self.manip.CalculateAngularVelocityJacobian()
      curLoc = self.robot.GetDOFValues(self.arm_indices)
      jacobianTranspose = False
      if jacobianTranspose:
        # get the joint change needed in direction of desired translation
        nextTransDiff = np.transpose(jac).dot(diffTrans) 
        # get the joint change needed in direction of desired rotation
        nextRotDiff = th.convert_axis_angle_to_joint(angVelJac, diffRotAxis, diffRotAngle)
        step = get_step(nextTransDiff, nextRotDiff)
      else:
        step = th.least_squares_step(jac, angVelJac, diffTrans, diffRotAxis * diffRotAngle) * 0.1
        desMaxChange = 0.4
        curMaxChange = max(abs(step))
        if curMaxChange > desMaxChange:
          rospy.logwarn("one of the joints was going too fast, so slowing motion")
          step = step*desMaxChange/curMaxChange
        if (np.any((curLoc + step) < self.joint_min) or 
           np.any((curLoc + step) > self.joint_max)):
          rospy.logwarn("The joint limits would have been exceeded, so not taking any step. Requested joints: %s, Joint minimum bounds: %s, Joint maximum bounds %s"%(curLoc + step, self.joint_min, self.joint_max))
          return
      traj = self.create_two_point_trajectory(curLoc + step)
      if constrainMotion:
        pass
     
    if self.should_perform_planned_move(curtrans[0:3,3], endLoc):
      rospy.logwarn("The target location was too far away, so we're using the slower planner to get there")
      self.move_to_target_by_planner(endLoc, constrainMotion=False) 
    else:
      #rospy.logwarn(traj) 
      self.robot.ExecuteTrajectory(traj)

  def create_two_point_trajectory(self,goal_joint_values):
    # manipulator has 6DOF, but we also need to specify the finger DOFs, apparently
    # so we get the full DOFs here by concatenating on two extra DOFs
    extraDOFValues = []
    curLoc = self.robot.GetDOFValues(self.arm_indices)
    #rospy.logwarn("curFullLoc is %s"%curLoc)
    traj = openravepy.RaveCreateTrajectory(self.env,'')
    traj.Init(self.robot.GetActiveConfigurationSpecification())
    traj.Insert(0,np.concatenate((curLoc, extraDOFValues)))
    traj.Insert(1,np.concatenate((goal_joint_values, extraDOFValues)))
    openravepy.planningutils.RetimeActiveDOFTrajectory(traj,self.robot)#,hastimestamps=False,maxvelmult=1,plannername='ParabolicTrajectoryRetimer')

    return traj
