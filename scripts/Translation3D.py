#!/usr/bin/python
"""Shows how to set a physics engine and send torque commands to the robot
"""
import rospy
import argparse
from openravepy import *
import time
import numpy, time
import adapy

rospy.init_node('adapy', anonymous=True)
parser = argparse.ArgumentParser(description='utility script for loading AdaPy')
parser.add_argument('-s', '--sim', action='store_true',
                        help='simulation mode')
parser.add_argument('-v', '--viewer', nargs='?', const=True,
                        help='attach a viewer of the specified type')
parser.add_argument('--env-xml', type=str,
                        help='environment XML file; defaults to an empty environment')
parser.add_argument('--debug', action='store_true',
                        help='enable debug logging')
args = parser.parse_args()

RaveInitialize(True)
misc.InitOpenRAVELogging()

env, robot = adapy.initialize(
    sim=args.sim,
    attach_viewer=args.viewer,
    env_path=args.env_xml
)

"""Moves a robot in a random position, gets the end effector transform, and calls IK on it.
"""

manip = robot.GetActiveManipulator()
ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Translation3D)
ikmodel.load()
#ikmodel.autogenerate()

with robot: # lock environment and save robot state
    robot.SetDOFValues([2.58, 0.547, -0.5, -0.7],[0,1,2,3]) # set the first 4 dof values
    Tee = manip.GetEndEffectorTransform() # get end effector
    ikparam = IkParameterization(Tee[0:3,3],ikmodel.iktype) # build up the translation3d ik query
    sols = manip.FindIKSolutions(ikparam, IkFilterOptions.CheckEnvCollisions) # get all solutions

h = env.plot3(Tee[0:3,3],10) # plot one point
with robot: # save robot state
    raveLogInfo('%d solutions'%len(sols))
    for sol in sols: # go through every solution
        robot.SetDOFValues(sol,manip.GetArmIndices()) # set the current solution
        env.UpdatePublishedBodies() # allow viewer to update new robot
        time.sleep(10.0/len(sols))

raveLogInfo('restored dof values: '+repr(robot.GetDOFValues())) # robot state is restored to original
