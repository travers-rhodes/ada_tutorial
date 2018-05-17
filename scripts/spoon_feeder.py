#!/usr/bin/env python
import rospy

import tracker_interface as tracker
from state_transition_logic import transitionLogicDictionary, State


class SpoonFeeder:
  def __init__(self):
    self.tracker = tracker.TrackerInterface()
    self.state = State.WAIT_EMPTY

    while not rospy.is_shutdown():
      transitionLogic = transitionLogicDictionary[self.state]()
      nextState = transitionLogic.wait_and_return_next_state()
      self._set_state(nextState)

  def _set_state(self, state):
    rospy.logwarn("State is now %s" % state)
    self.state = state 
    self._update_tracker_based_on_state()

  def _update_tracker_based_on_state(self):
    if (self.state == State.WAIT_EMPTY or
        self.state == State.WAIT_FULL or
        self.state == State.WAIT_WHILE_BITE): 
      self.tracker.stop_moving()
    elif self.state == State.PICK_UP_FOOD:
      self.tracker.stop_moving()
    elif self.state == State.MOVE_TO_MOUTH:
      self.tracker.start_updating_target_relative_to_mouth()
    elif self.state == State.MOVE_TO_PLATE: 
      self.tracker.start_tracking_fixed_target([0.3,-0.3,0.1])


if __name__=="__main__":
  rospy.init_node('spoon_feeder', anonymous=True)
  s = SpoonFeeder()
