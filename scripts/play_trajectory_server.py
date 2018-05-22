#!/usr/bin/env python
import rospy
import argparse
import numpy as np

import play_tapo_trajectory as ptt

from ada_tutorial.srv import PlayTrajectory, PlayTrajectoryResponse 

def main():
  # initialize the ros node 
  rospy.init_node('play_trajectory_server', anonymous=True)
  pts = PlayTrajectoryService()
  s = rospy.Service('play_trajectory', PlayTrajectory, pts.handle_play_request)
  pts.play_when_called()

class PlayTrajectoryService:
  def __init__(self):
    self.is_playing = False

  def play_when_called(self):
    # keep this loop from going faster than a fixed amount by means of rospy.rate
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
      try:
        if self.target_pose is not None:
          ptt.publish_poses("subject11_potato_salad/2.csv")
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
