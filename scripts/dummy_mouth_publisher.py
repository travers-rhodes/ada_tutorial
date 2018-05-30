#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped

point_topic = "/DO/inferenceOut/Point"
point_stamped_topic = point_topic + "Stamped"

rospy.init_node("dummy_publisher")
pub = rospy.Publisher(point_topic, Point, queue_size=10)
pubStamped = rospy.Publisher(point_stamped_topic, PointStamped, queue_size=10)
h = Header()
h.stamp = rospy.Time.now()
times = np.array(range(100))
# note that on this branch these are robot coordinates
x_dist = 0.50
y_dist = 0.15
z_dist = 0.30 
radius = 0

poses = [[ x_dist + radius * np.sin(t), y_dist + radius * np.cos(t), z_dist] for t in times]

for pose in poses+poses+poses:
  mesg = Point(pose[0], pose[1], pose[2])
  h = Header()
  h.stamp = rospy.Time.now()
  h.frame_id = "camera_rgb_optical_frame"
  mesgStamped = PointStamped(h,mesg)
  pub.publish(mesg)
  pubStamped.publish(mesgStamped)
  rospy.sleep(1)


