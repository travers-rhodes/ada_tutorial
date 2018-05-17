import rospy
from enum import Enum

from std_msgs.msg import Bool, Float64

class State(Enum):
  WAIT_EMPTY = 1
  PICK_UP_FOOD = 2
  WAIT_FULL = 3
  MOVE_TO_MOUTH = 4
  WAIT_WHILE_BITE = 5
  MOVE_TO_PLATE = 6

distance_to_goal_topic = "/distance_to_goal" # std_msgs/Float64
head_moving_topic = "/head_moving" # std_msgs/Bool
head_not_moving_topic = "/head_not_moving" # std_msgs/Bool
                       
class TransitionLogic:
  def __enter__(self):
    return self
  def __exit__(self, exc_type, exc_value, traceback):
    # if your extension creates ros subscribers, be sure to overwrite this 
    pass

  # this method subscribes to any relevant ros topics
  # blocks, and returns a new state when the robot should transition
  # from the current state to a new state.
  # This class should be extended by any logic implementation for each of the different states
  def wait_and_return_next_state(self):
    rospy.logerr("This method shouldn't be called from this base class, but should be implemented in the child class")

class EmptyStateTransitionLogic(TransitionLogic):
  def wait_and_return_next_state(self):
    rospy.logwarn("Waiting before picking up food")
    rospy.sleep(1)
    return State.PICK_UP_FOOD

class PickUpStateTransitionLogic(TransitionLogic):
  def wait_and_return_next_state(self):
    rospy.logwarn("Picking up food")
    rospy.sleep(1)
    return State.WAIT_FULL

class WaitFullStateTransitionLogic(TransitionLogic):
  def __init__(self):
    self.ready_for_bite = False
    self.listenForReady = rospy.Subscriber(head_not_moving_topic, Bool, self.update_head_ready)

  def __exit__(self, exc_type, exc_value, traceback):
    self.listenForReady.unregister()

  def wait_and_return_next_state(self):
    rospy.sleep(1) #temporary hack just wait a second then go
    return State.MOVE_TO_MOUTH
    
    r = rospy.Rate(10) # 10Hz
    while not self.ready_for_bite:
      r.sleep() 
    return State.MOVE_TO_MOUTH
  
  def update_head_ready(self, message):
    self.ready_for_bite = message.data

class MoveToMouthStateTransitionLogic(TransitionLogic):
  def __init__(self):
    self.distance_to_goal = None
    self.head_is_moving = False
    self.listenForCancel = rospy.Subscriber(head_moving_topic, Bool, self.update_head_moving)
    self.listenForSuccess = rospy.Subscriber(distance_to_goal_topic, Float64, self.update_distance_to_goal)
  
  def __exit__(self, exc_type, exc_value, traceback):
    self.listenForSuccess.unregister()
    self.listenForCancel.unregister()

  def wait_and_return_next_state(self):
    rospy.logwarn("Moving to mouth, judging completion by distance to mouth")
    r = rospy.Rate(10)
    epsilon_to_mouth = 0.05
    while not self.head_is_moving and (self.distance_to_goal is None or self.distance_to_goal > epsilon_to_mouth):
      r.sleep()
    if self.head_is_moving:
      rospy.logwarn("I noticed the head moved a bunch, so I'm going back to the plate to wait")
      return State.MOVE_TO_PLATE
    return State.WAIT_WHILE_BITE 
  
  def update_head_moving(self, message):
    self.head_is_moving = message.data
  def update_distance_to_goal(self, message):
    self.distance_to_goal = message.data

class WaitWhileBiteStateTransitionLogic(TransitionLogic):
  def wait_and_return_next_state(self):
    rospy.logwarn("Waiting near mouth for 5 seconds")
    rospy.sleep(5)
    return State.MOVE_TO_PLATE

class MoveToPlateStateTransitionLogic(TransitionLogic):
  def __init__(self):
    self.distance_to_goal = None
    self.listenForSuccess = rospy.Subscriber(distance_to_goal_topic, Float64, self.update_distance_to_goal)
    
  def __exit__(self, exc_type, exc_value, traceback):
    self.listenForSuccess.unregister()

  def wait_and_return_next_state(self):
    rospy.logwarn("Moving back to plate")
    r = rospy.Rate(10)
    epsilon_to_plate = 0.05
    while (self.distance_to_goal is None or self.distance_to_goal > epsilon_to_plate):
      r.sleep()
    return State.WAIT_EMPTY 
  
  def update_distance_to_goal(self, message):
    self.distance_to_goal = message.data

# a dictionary that gives you the logic class constructor associated with each state
transitionLogicDictionary = { 
                     State.WAIT_EMPTY      : EmptyStateTransitionLogic,
                     State.PICK_UP_FOOD    : PickUpStateTransitionLogic,
                     State.WAIT_FULL       : WaitFullStateTransitionLogic,
                     State.MOVE_TO_MOUTH   : MoveToMouthStateTransitionLogic,
                     State.WAIT_WHILE_BITE : WaitWhileBiteStateTransitionLogic,
                     State.MOVE_TO_PLATE   : MoveToPlateStateTransitionLogic }

