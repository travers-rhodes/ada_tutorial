#!/usr/bin/python
"""
Use gradient descent to move the hand to the cartesian target
"""
import rospy
import numpy as np
import openravepy

import adapy

import transform_helpers as th
import transforms3d as t3d
from std_msgs.msg import Float64
 
class AdaJacobianControl():
  def __init__(self, args, endEffName="Spoon"): 
    ###
    ### CONSTANTS that we might turn into parameters later
    ###
    # what is the fraction of the max speed we allow motors to go
    self.velocity_fraction = 1
    # how far can the end-effector move on each step
    self.transStepSize = 0.05
    self.angleStepSize = 0.1
    self.transEpsilon = 0.001
    # this one's a weird number, but it's a measure of how far off the rotation can get before we try to correct
    self.quatEpsilon = 0.001
    # what is the largest number of radians we allow a motor to rotate in each linear step
    self.max_rotation_per_step = 0.4
    self.distance_pub = rospy.Publisher("/distance_to_target", Float64, queue_size=10)
    ###
    ### Initialize robot
    ###
    self.initialize_robot(args, endEffName)
    
  def initialize_robot(self, args, endEffName):
    openravepy.RaveInitialize(True)
    openravepy.misc.InitOpenRAVELogging()
    self.env, self.robot = adapy.initialize(
        sim=args.sim,
        attach_viewer=args.viewer,
        env_path=args.env_xml
    )
    self.manip = self.robot.SetActiveManipulator(endEffName)
    # even though it's not obvious how we use this, we need to initialize the IKModel on self.robot
    ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(self.robot,iktype=openravepy.IkParameterization.Type.Transform6D)
    #ikmodel.autogenerate()
    ikmodel.load()
    vel_limits = self.robot.GetDOFVelocityLimits()
    self.robot.SetDOFVelocityLimits(vel_limits * self.velocity_fraction)
    self.arm_indices = self.manip.GetArmIndices()
    self.robot.SetActiveDOFs(self.arm_indices)
    self.joint_min, self.joint_max = self.robot.GetDOFLimits(self.arm_indices)
 
  # if we aren't already at target, pick a pseudo target on the way to the target
  # and head to that pseudo target 
  def make_step_to_target_pose(self, endPose):
    endLoc = np.array([endPose.position.x,
                       endPose.position.y,
                       endPose.position.z])
    endQuat = np.array([endPose.orientation.w, 
                        endPose.orientation.x, 
                        endPose.orientation.y, 
                        endPose.orientation.z])
    curtrans = self.manip.GetEndEffectorTransform()
    curCartLoc = curtrans[0:3,3]
    quat_dist = th.quat_distance(t3d.quaternions.mat2quat(curtrans[0:3,0:3]), endQuat)
    #rospy.logwarn("quat_dist is %s"%quat_dist)
    # don't do anything if close enough to target
    if (th.distance(curCartLoc, endLoc) < self.transEpsilon and
       # quat_distance is between 0 and 1
       quat_dist < self.quatEpsilon):
      return
    pseudoEndLoc = self.get_pseudo_endLoc(curCartLoc, endLoc)
    self.make_step_to_pseudotarget(pseudoEndLoc, endQuat)
 
  def make_step_to_pseudotarget(self, endLoc, endQuat):
    with self.env:
      curtrans = self.manip.GetEndEffectorTransform()
      diffTrans, diffRotAxis, diffRotAngle = th.get_transform_difference_axis_angle(curtrans, endLoc, endQuat)
      jac= self.manip.CalculateJacobian()
      angVelJac = self.manip.CalculateAngularVelocityJacobian()
      curLoc = self.robot.GetDOFValues(self.arm_indices)
      transDist = th.distance(diffTrans,[0,0,0])
      self.distance_pub.publish(Float64(transDist))
      if transDist > self.transStepSize:
        stepScale = self.transStepSize / transDist
        diffTrans = diffTrans * stepScale
        diffRotAngle = diffRotAngle * stepScale 
        rospy.logwarn("Translation was larger than transStepSize, so only going %f of the way"%stepScale)
      if diffRotAngle > self.angleStepSize:
        angScale = self.angleStepSize / diffRotAngle
        diffTrans = diffTrans * angScale 
        diffRotAngle = diffRotAngle * angScale 
        rospy.logwarn("Rotation was larger than angleStepSize, so only going %f of the way"%angScale)
      step = th.least_squares_step(jac, angVelJac, diffTrans, diffRotAxis * diffRotAngle) 
      desMaxChange = self.max_rotation_per_step
      curMaxChange = max(abs(step))
      if curMaxChange > desMaxChange:
        rospy.logwarn("one of the joints was going too fast, so slowing motion")
        step = step*desMaxChange/curMaxChange
      if (np.any((curLoc + step) < self.joint_min) or 
          np.any((curLoc + step) > self.joint_max)):
        rospy.logwarn("The joint limits would have been exceeded, so not taking any step. Requested joints: %s, Joint minimum bounds: %s, Joint maximum bounds %s"%(curLoc + step, self.joint_min, self.joint_max))
        return
      traj = self.create_two_point_trajectory(curLoc + step)
    self.robot.ExecuteTrajectory(traj)

  # todo: clean this up. But in general, if you're moving large distances in y,
  # then you should stay away from the pivot point (increase your x) 
  def get_pseudo_endLoc(self, curCartLoc, endLoc):
    #rospy.logwarn("CURRENT POSITION is %s" % curCartLoc)
    breaks = [-0.15, -0.07, 0, 0.07, 0.15]
    lenMinusOne = len(breaks) - 1
    pseudoEndLoc = np.array(endLoc)
    for i in range(lenMinusOne):
      if curCartLoc[1] < breaks[i] and endLoc[1] > breaks[i+1]:
        pseudoEndLoc[1] = (breaks[i+1] + breaks[i])/2.0
        if endLoc[1] - curCartLoc[1] > 0.15:
          pseudoEndLoc[0] = endLoc[0] + 0.1
      elif curCartLoc[1] > breaks[i+1] and endLoc[1] < breaks[i]:
        pseudoEndLoc[1] = (breaks[i+1] + breaks[i])/2.0
        pseudoEndLoc[0] = endLoc[0] + 0.2
    #rospy.logwarn("pseudoEndLoc is %s"% pseudoEndLoc)
    return pseudoEndLoc

  def create_two_point_trajectory(self,goal_joint_values):
    curLoc = self.robot.GetDOFValues(self.arm_indices)
    traj = openravepy.RaveCreateTrajectory(self.env,'')
    traj.Init(self.robot.GetActiveConfigurationSpecification())
    traj.Insert(0,curLoc)
    traj.Insert(1,goal_joint_values)
    openravepy.planningutils.RetimeActiveDOFTrajectory(traj,self.robot)#,hastimestamps=False,maxvelmult=1,plannername='ParabolicTrajectoryRetimer')
    return traj
