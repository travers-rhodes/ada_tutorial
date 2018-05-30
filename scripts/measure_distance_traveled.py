import rospy
import transform_helpers as th
import numpy as np

from geometry_msgs.msg import PointStamped

class TrackDistance:
  def __init__(self):
    self.cumulative_distance = 0
    self.is_active = False
    self.last_position = None
    self.track_distance_listener = rospy.Subscriber("/current_location", PointStamped, self.current_position_callback)

  def current_position_callback(self, msg):
    if not self.is_active:
      #rospy.logwarn("Distance measure not active")
      return 
    curLoc = np.array([msg.point.x, msg.point.y, msg.point.z])
    if self.last_position is None:
      # set the current position as last position and return
      self.last_position = curLoc
      return
    current_distance = th.distance(curLoc, self.last_position)
    self.cumulative_distance += current_distance
    self.last_position = curLoc
    #rospy.logwarn("Update distance traveled to %s"%self.cumulative_distance)

  def start(self):
    self.is_active = True

  def stop(self):
    self.is_active = False
    self.last_position = None
    
