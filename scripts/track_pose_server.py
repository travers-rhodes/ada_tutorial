#!/usr/bin/env python
import rospy
import argparse
import numpy as np

from ada_tutorial.srv import TrackPose, TrackPoseResponse
from ada_jacobian_control import AdaJacobianControl

def main(args):
  # initialize the ros node 
  rospy.init_node('track_pose_server', anonymous=True)
  tas = TrackPoseService(args)
  s = rospy.Service('update_pose_target', TrackPose, tas.handle_target_update)
  tas.run_tracking()

class TrackPoseService:
  def __init__(self, args):
    self.ada_control = AdaJacobianControl(args,endEffName="Spoon")
    self.target_pose = None

  def run_tracking(self):
    # keep this loop from going faster than a fixed amount by means of rospy.rate
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
      try:
        if self.target_pose is not None:
          self.ada_control.make_step_to_target_pose(self.target_pose)
      except Exception as e:
        rospy.logerr(e)
        # we don't want this service to ever actually throw errors and fail out
        raise
      # We don't need special code to allow any callbacks to run, in case the user has updated the location
      # since in rospy, callbacks are always called in separate threads 
      # however, since sometimes the loop is a no-op, we add a sleep to keep it from going faster than 10hz
      r.sleep()

  # takes in a TrackArm request and calls ada_control 
  # to move the arm based on that request
  def handle_target_update(self, req):
    # move_to_target's endLoc should be a length 3 np.array
    # of the coordinates to move the end-effector of the arm to in
    # cartesian coordinates relative to the base frame of the arm
    isSuccess = True
    if req.stopMotion:
      self.pose = None
      return TrackPoseResponse(isSuccess)
    self.target_pose = req.target 
    return TrackPoseResponse(isSuccess)

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
