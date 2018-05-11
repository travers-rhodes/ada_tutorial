#!/usr/bin/env python
import rospy
import argparse
import numpy as np

from ada_tutorial.srv import MoveArm, MoveArmResponse
from ada_cartesian_control import AdaCartesianControl
from ada_jacobian_control import AdaJacobianControl

def main(args):
  # initialize the ros node 
  rospy.init_node('move_arm_server', anonymous=True)
  mas = MoveArmService(args)
  s = rospy.Service('move_arm', MoveArm, mas.handle_move_arm)
  rospy.spin() 

class MoveArmService:
  def __init__(self, args):
    #self.ada_control = AdaCartesianControl(args)
    self.ada_control = AdaJacobianControl(args)

  # takes in a MoveArm request and calls ada_control 
  # to move the arm based on that request
  def handle_move_arm(self, req):
    # move_to_target's endLoc should be a length 3 np.array
    # of the coordinates to move the end-effector of the arm to in
    # cartesian coordinates relative to the base frame of the arm
    self.ada_control.move_to_target(endLoc=np.array([req.target.x, req.target.y, req.target.z]), constrainMotion=req.constrainMotion)
    return MoveArmResponse(True)


if __name__=="__main__":
  # parse input arguments
  parser = argparse.ArgumentParser(description='node to track the position given by a rostopic')
  parser.add_argument('-s', '--sim', action='store_true',
                          help='simulation mode')
  parser.add_argument('-v', '--viewer', nargs='?', const=True,
                          help='attach a viewer of the specified type')
  parser.add_argument('--env-xml', type=str,
                          help='environment XML file; defaults to an empty environment')
  parser.add_argument('--debug', action='store_true',
                          help='enable debug logging')
  args = parser.parse_args(rospy.myargv()[1:])
  main(args) 
 
  
   
  
  
