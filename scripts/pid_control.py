#!/usr/bin/python
"""Shows how to set a physics engine and send torque commands to the robot
"""
import rospy
import argparse
from openravepy import *
import numpy as np
import time
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

manip = robot.GetActiveManipulator()
ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Translation3D)
ikmodel.load()

######
# relevant tuneable parameters
######
debug = args.debug
Kp = 0
Kd = 0


def get_target_joints(loc = None):
  if (loc is None):
    Tee = manip.GetEndEffectorTransform()
    loc = Tee[0:3,3]
    if (debug):
      print("Computed end effector location to be %s" % loc)
  ikparam = IkParameterization(loc, ikmodel.iktype) # build up the translation3d ik query
  sol = manip.FindIKSolution(ikparam, IkFilterOptions.CheckEnvCollisions)
  # fix an offset issue due to the fact that this joint has no joint limits
  while sol[5] < 0:
    sol[5] += 2 * np.pi
  while sol[5] > 2 * np.pi:
    sol[5] -= 2 * np.pi
  if (debug):
    print("current joint angles are %s" % robot.GetDOFValues())
    print("Joint angles desired are %s" % sol)
  return sol

def get_pid_torques(target_joints, M):
  curState = robot.GetDOFValues()
  curStateDot = robot.GetDOFVelocities()
  # note the need to add two extra dimensions (because of finger dofs?)
  error = curState - np.concatenate((target_joints , [0.0,0.0]))
  errorDot = curStateDot
  if debug:
    print("Current error is %s" % error)
    print("Current errorDot is %s" % errorDot)
  mintorque = -1
  maxtorque = 1
  return np.array([max(mintorque, min(maxtorque, t)) for t in M.dot(- Kp * error - Kd * errorDot)])

def get_M_matrix():
  M = [] 
  for i in range(robot.GetDOF()): 
    testaccel = np.zeros(robot.GetDOF()) 
    testaccel[i] = 1.0 
    M.append(robot.ComputeInverseDynamics(dofaccelerations=testaccel, externalforcetorque=None, returncomponents=True)[0]) 
  M = np.array(M)
  if (debug):
    print("Mass matrix is %s" % M)
  return(M)

def get_G_torques():
  (M, C, G) = robot.ComputeInverseDynamics(dofaccelerations=None, externalforcetorque=None, returncomponents=True) 
  if debug:
    print("G is ", G)
  return(G)

with env:
    # not sure why we need to load this here instead of load this as an
    # arg to this script (before loading the robot into the scene), but if
    # you were to load this as an arg to this script, the robot wouldn't stay
    # fixed to the grid and would float to the ground
    env.Load("./config/tutorial.env.xml")
    # you could use the below commented out code to set gravity, but I
    # don't see an easy way to set the ERP parameter
    #physics = RaveCreatePhysicsEngine(env,'ode')
    #env.SetPhysicsEngine(physics)
    #physics.SetGravity(numpy.array((0,0,-9.8)))
    
    
    robot = env.GetRobots()[0]
    robot.GetLinks()[0].SetStatic(True)
    env.StopSimulation()
    env.StartSimulation(timestep=0.001)

pid_rate = rospy.Rate(300) # update controller 100Hz
inertia_rate_ratio = 1 # update inertia matrix after every 100 updates of pid_rate
target_joints = get_target_joints()
M  = get_M_matrix() 
G = get_G_torques() * 4
iterCount = 0
while not rospy.is_shutdown(): 
  if iterCount % inertia_rate_ratio == 0:
    M = get_M_matrix() 
    G = get_G_torques() * 3.5
  torques = get_pid_torques(target_joints, M) + G
  # have to lock environment when calling robot methods
  with env:
    if debug and False:
      print("torques are %s" % torques)
    robot.SetDOFTorques(torques,True)
  pid_rate.sleep()
  iterCount += 1

# reset the physics engine
env.SetPhysicsEngine(None)
