#!/usr/bin/python
"""Shows how to set a physics engine and send torque commands to the robot
"""
import rospy
import numpy as np
import time
import adapy
import argparse
import tf

from ada_tutorial.srv import MoveArm
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped

point_topic = "/DO/inferenceOut/Point"
point_stamped_topic = point_topic + "Stamped"
x_dist = 0.41676946

class CameraCalibration:
  def __init__(self):
    camera_to_camera = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    self.camera_to_robot =  np.array([[1,0,0,0.4],[0,0,1,-0.8],[0,-1,0,0.30],[0,0,0,1]])
    self.br = tf.TransformBroadcaster()
    rospy.logwarn("camera calibration initialized")
    
    rospy.Timer(rospy.Duration(0.01), self.broadcastTransform)
    rospy.logwarn("sent first message")

  def broadcastTransform(self, event):
    #rospy.logwarn("broadcasting tf")
    self.br.sendTransform(self.camera_to_robot[0:3,3],
                          # note we pass in 4x4 to this method... https://github.com/ros/geometry/issues/64
                          tf.transformations.quaternion_from_matrix(self.camera_to_robot),
                          rospy.Time.now(),
                          "camera_rgb_optical_frame",
                          "odom")

  # point should be a length 4 np.array giving the location of the target point in the camera frame
  def convert_to_robot_frame(self, point):
    return self.camera_to_robot.dot(point.transpose())

class Tracker:
  def __init__(self):
    self.cameraCalib = CameraCalibration()

    rospy.wait_for_service('update_track_target', timeout=None)
    self.update_track_target = rospy.ServiceProxy('update_track_target', MoveArm)
    self.target_listener = rospy.Subscriber(point_topic, Point, self.track_target)
    self.pubStamped = rospy.Publisher(point_stamped_topic, PointStamped, queue_size=10)

  # return an np.array of the [x,y,z] target mouth location
  # in the coordinate frame of the robot base
  def get_target_loc(self, mouth_pos):
    t = mouth_pos
    point_in_camera_frame = np.array([t.x, t.y, t.z, 1])
    point_in_robot_frame = self.cameraCalib.convert_to_robot_frame(point_in_camera_frame)
    return point_in_robot_frame[0:3]

  # compute and move toward the target mouth location
  # only move for at most timeoutSecs,   
  # if constrainMotion is set to False, don't allow the robot end effector to rotate, and only allow linear motion toward the goal
  # otherwise, don't constrain motion
  def track_target(self, mouth_pos):
    endLoc = self.get_target_loc(mouth_pos) 
    h = Header()
    h.stamp = rospy.Time.now()
    h.frame_id = "camera_rgb_optical_frame"
    self.pubStamped.publish(PointStamped(h,mouth_pos))
    self.update_track_target(target=Point(endLoc[0], endLoc[1], endLoc[2]), constrainMotion=True)
    

if __name__=="__main__":
  # parse input arguments
  parser = argparse.ArgumentParser(description='node to track the position given by a rostopic')
  parser.add_argument('-s', '--sim', action='store_true',
                          help='simulation mode')
  args = parser.parse_args(rospy.myargv()[1:])
  # initialize the ros node 
  rospy.init_node('adapy_tracker', anonymous=True)
  t = Tracker()

  rospy.logwarn("fast_tracker args are %s"% args)
  if (not args.sim):
    rospy.logwarn("spinning")
    rospy.spin()

  # reset the camera-to-robot translation to a trivial one
  t.cameraCalib.camera_to_robot=np.eye(4)
  pub = rospy.Publisher(point_topic, Point, queue_size=10)
  pubStamped = rospy.Publisher(point_stamped_topic, PointStamped, queue_size=10)
  h = Header()
  h.stamp = rospy.Time.now()
  times = np.array(range(100))
  y_center = -0.195
  z_center = 0.45
  poses = [[ x_dist, y_center + 0.2 * np.sin(t),z_center + 0.2 * np.cos(t)] for t in times]

  for pose in poses+poses+poses:
    mesg = Point(pose[0], pose[1], pose[2])
    h = Header()
    h.stamp = rospy.Time.now()
    h.frame_id = "odom"
    mesgStamped = PointStamped(h,mesg)
    rospy.logwarn("publishing pose")
    pub.publish(mesg)
    rospy.logwarn("publishing pose")
    pubStamped.publish(mesgStamped)
    rospy.logwarn("publishing pose")
    #print("GOING TO MOVE") 
    #track.move_to_target(target=Point(pose[0], pose[1], pose[2]), constrainMotion=True)
    #print("Service Returned") 
    rospy.sleep(4)
  thread.join()


