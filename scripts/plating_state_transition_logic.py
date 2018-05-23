import rospy
from enum import Enum

from std_msgs.msg import Bool, Float64

class State(Enum):
  PICK_UP_FOOD = 1
  DROP_OFF_FOOD = 2

food_acquired_topic = "/food_acquired" # std_msgs/Bool
                       
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

class PickUpStateTransitionLogic(TransitionLogic):
  def __init__(self):
    self.food_acquired = False
    self.listenForFoodAcquired = rospy.Subscriber(food_acquired_topic, Bool, self.update_food_acquired)

  def __exit__(self, exc_type, exc_value, traceback):
    self.listenForFoodAcquired.unregister()

  def wait_and_return_next_state(self):
    rospy.logwarn("Picking up food")
    r = rospy.Rate(10) # 10Hz
    while not self.food_acquired:
      r.sleep() 
    return State.DROP_OFF_FOOD
  
  def update_food_acquired(self, message):
    self.food_acquired = message.data

class DropOffStateTransitionLogic(TransitionLogic):
  def __init__(self):
    self.food_dropped = False
    self.listenForFoodDropped= rospy.Subscriber(food_acquired_topic, Bool, self.update_food_dropped)

  def __exit__(self, exc_type, exc_value, traceback):
    self.listenForFoodDropped.unregister()

  def wait_and_return_next_state(self):
    rospy.logwarn("Picking up food")
    r = rospy.Rate(10) # 10Hz
    while not self.food_dropped:
      r.sleep() 
    return State.PICK_UP_FOOD
  
  def update_food_dropped(self, message):
    self.food_dropped = message.data

# a dictionary that gives you the logic class constructor associated with each state
transitionLogicDictionary = { 
                     State.PICK_UP_FOOD    : PickUpStateTransitionLogic,
                     State.DROP_OFF_FOOD    : DropOffStateTransitionLogic,
                     }

