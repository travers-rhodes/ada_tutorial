#!/usr/bin/env python
import rospy
import rospkg
import numpy as np
from threading import Thread

import tracker_interface as tracker
import measure_distance_traveled 
# last_hist_corr and last_file_name are super hacky ways to remember the info about most recent images taken
from feeding_state_transition_logic import transitionLogicDictionary, State, mostRecentImage
from ada_tutorial.srv import PlayTrajectory
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion


class SpoonFeeder:
  def __init__(self):
    # set seed for slightly more reproducibility
    np.random.seed(30)
    self.defaultQuat = Quaternion(0.5, 0.5, 0.5, 0.5)
    self.tracker = tracker.TrackerInterface(self.defaultQuat)
    self.distance_tracker = measure_distance_traveled.TrackDistance()
    self.play_trajectory_topic = "/Tapo/example_poses"
    self._play_trajectory = rospy.ServiceProxy("play_trajectory", PlayTrajectory)
    rospy.logwarn("TrackerInterface successfully initialized")
    # annoying problem here, but basically we don't want to count the first move
    # to the plate as distance traveled. But we do want to count all future moves
    # to the plate
    self.is_first_move_to_plate = True
    rospack = rospkg.RosPack()
    self.results_file = rospack.get_path("ada_tutorial") + "/scale_weights.txt"
    self._set_state(State.MOVE_TO_PLATE)

    while not rospy.is_shutdown():
      with transitionLogicDictionary[self.state]() as transitionLogic:
        rospy.logwarn("About to wait and return")
        nextState = transitionLogic.wait_and_return_next_state() 
      rospy.logwarn("returned")
      self._set_state(nextState)

  def _set_state(self, state):
    rospy.logwarn("State is now %s" % state)
    self.state = state 
    self._update_tracker_based_on_state()

  def _update_tracker_based_on_state(self):
    if self.state == State.MOVE_TO_PLATE: 
      if not self.is_first_move_to_plate:
        self.distance_tracker.start()
      self.tracker.start_tracking_fixed_target([0.3,-0.3,0.1])
      self.is_first_move_to_plate = False
    elif self.state == State.PICK_UP_FOOD:
      rospy.logwarn("You have traveled %s kilometers."% self.distance_tracker.cumulative_distance)
      # distance_tracker.start() is a projection so it's fine to call it more than once
      self.distance_tracker.start()
      self.xoffset = np.random.uniform() * 0.06 #0.03
      self.yoffset = np.random.uniform() * 0.03 #0.015
      #self.zoffset = -np.random.uniform() * 0.02
      self.zoffset = -0.04
      self.tracker.start_updating_target_to_pose(self.play_trajectory_topic,[self.xoffset, self.yoffset, self.zoffset])
      self._play_trajectory(String(self.play_trajectory_topic))
    elif self.state == State.MOVE_TO_MOUTH:
      mouth_point_topic = "/DO/inferenceOut/Point"
      self.tracker.start_updating_target_to_point(mouth_point_topic)
    elif self.state == State.MOVE_TO_SCALE: 
      self.distance_tracker.stop()
      self.tracker.start_tracking_fixed_target([0.5,-0.1,0.1])
    elif self.state == State.DUMP_ON_SCALE:
      self.distance_tracker.stop()
      self.tracker.start_updating_target_to_pose(self.play_trajectory_topic,[0.1, 0.2, 0.03])
      self._play_trajectory(String(self.play_trajectory_topic))
    elif self.state == State.WAIT_FOR_WEIGHT_INPUT:
      # this is very naughty blocking code, but it's time to start running experiments
      # and blocking code here won't hurt anyone
      if True:
        rospy.logwarn("You have traveled %s kilometers."% self.distance_tracker.cumulative_distance)
        rospy.logwarn("Please input the current weight on the scale:")
        scale_weight = raw_input()
        with open(self.results_file, "a") as f:
          # pulling these static variables from the feeding_state_transition module is also very naughty
          f.write("%s, %s, %s, %s, %s, %s\n"%(rospy.Time.now().to_sec(), scale_weight, self.distance_tracker.cumulative_distance, mostRecentImage.number_scoops_total, mostRecentImage.last_hist_corr, mostRecentImage.last_file_name))
    elif self.state == State.MOVE_BACK_TO_MOUTH:
      mouth_point_topic = "/DO/inferenceOut/Point"
      self.tracker.start_updating_target_to_point(mouth_point_topic)
    else:
      rospy.logerr("The state %s is not known"%self.state)


if __name__=="__main__":
  rospy.init_node('spoon_feeder', anonymous=True)
  s = SpoonFeeder()
