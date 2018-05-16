#!/usr/bin/env python
import rospy
import argparse
import numpy as np

from ada_tutorial.srv import MoveArm, MoveArmResponse
from ada_jacobian_control import AdaJacobianControl

def main(args):
  # initialize the ros node 
  rospy.init_node('track_arm_server', anonymous=True)
  tas = TrackArmService(args)
  s = rospy.Service('update_track_target', MoveArm, tas.handle_target_update)
  tas.RunTracking()
  rospy.spin() 

class TrackArmService:
  def __init__(self, args):
    self.ada_control = AdaJacobianControl(args,endEffName="Spoon")
    self.target = None

  def RunTracking(self):
    while not rospy.is_shutdown():
      try:
        if self.target is not None:
          self.ada_control.make_step_to_target(self.target, constrainMotion=False)
      except Exception as e:
        rospy.logerr(e)
        # we don't want this service to ever actually throw errors and fail out
        #raise
      # We don't need special code to allow any callbacks to run, in case the user has updated the location
      # since in rospy, callbacks are always called in separate threads 

  # takes in a MoveArm request and calls ada_control 
  # to move the arm based on that request
  def handle_target_update(self, req):
    # move_to_target's endLoc should be a length 3 np.array
    # of the coordinates to move the end-effector of the arm to in
    # cartesian coordinates relative to the base frame of the arm
    isSuccess = True
    self.target = [req.target.x, req.target.y, req.target.z]
    return MoveArmResponse(isSuccess)

if __name__=="__main__":
  # parse input arguments
  parser = argparse.ArgumentParser(description='service server node to move arm to a given position')
  parser.add_argument('-s', '--sim', action='store_true',
                          help='simulation mode')
  parser.add_argument('-j', '--useJacobian', action='store_true',
                          help='simulation mode')
  parser.add_argument('-v', '--viewer', nargs='?', const=True,
                          help='attach a viewer of the specified type')
  parser.add_argument('--env-xml', type=str,
                          help='environment XML file; defaults to an empty environment')
  parser.add_argument('--debug', action='store_true',
                          help='enable debug logging')
  args = parser.parse_args(rospy.myargv()[1:])
  main(args) 
 
  
   
  
  
