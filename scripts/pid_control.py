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
Kp = 1
Kd = 0.9


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

def get_pid_torques(target_joints, previous_error, previous_time):
  curTime = rospy.Time.now() 
  curState = robot.GetDOFValues()
  curStateDot = robot.GetDOFVelocities()
  # note the need to add two extra dimensions (because of finger dofs?)
  error = curState - np.concatenate((target_joints , [0.0,0.0]))
  dur = (curTime - previous_time).to_sec()
  if debug:
    print("times are", curTime, previous_time, dur)
  if dur > 0 and previous_error is not None:
    errorDot = (error - previous_error)/dur
  else:
    errorDot = np.zeros(error.shape)
  if debug:
    print("Current error is %s" % error)
    print("Current errorDot is %s" % errorDot)
  mintorque = -100
  maxtorque = 100
  return (np.array([min(maxtorque, max(mintorque, t)) for t in - Kp * error - Kd * errorDot]),
      error,
      curTime)
with env:
    # not sure why we need to load this here instead of load this as an
    # arg to this function (before loading the robot into the scene), but at least it works!
    env.Load("./config/tutorial.env.xml")
    # (or, you could do it programatically set a physics engine
    #physics = RaveCreatePhysicsEngine(env,'ode')
    #env.SetPhysicsEngine(physics)
    #physics.SetGravity(numpy.array((0,0,-9.8)))
    
    
    robot = env.GetRobots()[0]
    robot.GetLinks()[0].SetStatic(True)
    env.StopSimulation()
    env.StartSimulation(timestep=0.001)

pid_rate = rospy.Rate(100) # update controller 100Hz
target_joints = get_target_joints()
previous_error = None
previous_time = rospy.Time.now()
while not rospy.is_shutdown(): 
  (torques, previous_error, previous_time) = get_pid_torques(target_joints, previous_error, previous_time)
  # have to lock environment when calling robot methods
  with env:
    if debug:
      print("torques are %s" % torques)
    robot.SetDOFTorques(torques,add=False)
  pid_rate.sleep()

# reset the physics engine
env.SetPhysicsEngine(None)
