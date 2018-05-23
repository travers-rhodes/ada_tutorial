#!/usr/bin/env python
import rospy
import numpy as np
from threading import Thread

import tracker_interface as tracker
from plating_state_transition_logic import transitionLogicDictionary, State
from ada_tutorial.srv import PlayTrajectory
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion


class RicePlater:
  def __init__(self):
    self.defaultQuat = Quaternion(1/np.sqrt(2), 0, 0, 1/np.sqrt(2))
    self.tracker = tracker.TrackerInterface(self.defaultQuat)
    self.play_trajectory_topic = "/Tapo/example_poses"
    self._play_trajectory = rospy.ServiceProxy("play_trajectory", PlayTrajectory)
    rospy.logwarn("TrackerInterface successfully initialized")
    self.offset = 0
    self._set_state(State.PICK_UP_FOOD)
    # easy hack to make the spoon move slightly every time it picks up food new

    while not rospy.is_shutdown():
      transitionLogic = transitionLogicDictionary[self.state]()
      rospy.logwarn("About to wait and return")
      nextState = transitionLogic.wait_and_return_next_state() 
      rospy.logwarn("returned")
      self._set_state(nextState)

  def _set_state(self, state):
    rospy.logwarn("State is now %s" % state)
    self.state = state 
    self._update_tracker_based_on_state()

  def _update_tracker_based_on_state(self):
    if self.state == State.PICK_UP_FOOD:
      if self.offset == 0:
        self.offset = 0.05
      else:
        self.offset = 0
      self.tracker.start_updating_target_to_pose(self.play_trajectory_topic,[self.offset, 0, 0])
      self._play_trajectory(String(self.play_trajectory_topic))
    elif self.state == State.DROP_OFF_FOOD:
      self.tracker.start_updating_target_to_pose(self.play_trajectory_topic,[0.2,0.2,0])
      self._play_trajectory(String(self.play_trajectory_topic))
    else:
      rospy.logerr("The state %s is not known"%self.state)


if __name__=="__main__":
  rospy.init_node('rice_plater', anonymous=True)
  s = RicePlater()
