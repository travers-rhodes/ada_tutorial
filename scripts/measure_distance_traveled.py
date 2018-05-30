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
      rospy.logwarn("Distance measure not active")
      return 
    if self.last_position is None:
      rospy.logwarn("Start timer at point")
      self.last_position = msg.point
      return
    current_distance = th.distance(np.array([msg.point.x, msg.point.y, msg.point.z]), 
                                   np.array([self.last_position.x, self.last_position.y, self.last_position.z]))
    self.cumulative_distance += current_distance
    rospy.logwarn("Update distance traveled to %s"%self.cumulative_distance)

  def start(self):
    self.is_active = True

  def stop(self):
    self.is_active = False
    self.last_position = None
    
