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

manip = robot.SetActiveManipulator("Mico")
print(manip)
ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
if not ikmodel.load():
  ikmodel.autogenerate()
print("loaded ik model")
######
# relevant tuneable parameters
######
debug = args.debug

roty = t3d.quaternions.quat2mat([0,np.sqrt(2), 0, np.sqrt(2)])
rotx = t3d.quaternions.quat2mat([np.sqrt(2),0, 0, np.sqrt(2)])
rot = roty.dot(rotx)
print("rot is %s" % rot)

poses =[ 
                [[ 0.21676946, -0.2959789,   0.552686472]],
                [[ 0.21676946, -0.2959789,   0.52686472]],
                [[ 0.21676946, -0.2959789,   0.42686472]],
                [[ 0.31676946, -0.1959789,   0.42686472]],
                [[ 0.31676946, -0.3959789,   0.42686472]]]

for i,endLoc in enumerate(poses+poses+poses):
  with env:
    manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs
    Tgoal = np.concatenate((np.concatenate((rot, np.transpose(endLoc)), axis=1),[[0,0,0,1]]), axis=0)
    constrainttaskmatrix=np.eye(4)
    constraintmatrix = np.linalg.inv(manip.GetTransform())
    constrainterrorthresh = 0.005 # copied from tutorial
    constraintfreedoms = [0,0,0,0,0,0] # based on parsing the tutorial. Not sure why it's not 0,0,1,...
    if i > 0:
     # don't let any x,y rotations happen
     constraintfreedoms = [1,1,0,0,0,0] 
    print("planning move")
    manipprob.MoveToHandPosition(matrices=[Tgoal],maxiter=3000,maxtries=1,seedik=40,
      constraintfreedoms=constraintfreedoms,
      constraintmatrix=constraintmatrix, 
      constrainttaskmatrix=constrainttaskmatrix,
      constrainterrorthresh=constrainterrorthresh,
      steplength=0.002)
  print("moving")
  robot.WaitForController(0) # wait
  print("moved to", Tgoal)
print("moving done")
rospy.spin()



