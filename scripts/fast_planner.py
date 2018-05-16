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
    """
    Copied from output from Alexandre's calibration. TODO put this in a parameter server
    'Rotation is: ', array([[ 0.99788558,  0.01727208, -0.06265819],
          [ 0.06467983, -0.16904555,  0.98348367],
          [ 0.00639472, -0.98545689, -0.16980528]]))
    ('Translation is:', array([[ 0.44780474],
          [-0.95989491],
          [ 0.43955628]]))
    """
    self.camera_to_robot = np.array([[0.99788558,  0.01727208, -0.06265819, 0.44780474],
                                     #[ 0.06467983, -0.16904555,  0.98348367, -0.95989491],
                                     [ 0.06467983, -0.16904555,  0.98348367, -0.91989491],
                                     [ 0.00639472, -0.98545689, -0.16980528, 0.43955628],
                                     [ 0, 0, 0, 1]])
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
                          "world")

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
    # if we're in preparation mode, we move to the point 30 cm in front of the face.
    # when we're not in preparation mode, we move to the mouth
    self.inPreparationMode = True

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
    # if we're in preparation mode, then update the z-location to be 30 cm less than actual mouth
    safetyDistance = -0.30 if self.inPreparationMode else 0  
    self.update_track_target(target=Point(endLoc[0], endLoc[1]+safetyDistance, endLoc[2]), constrainMotion=True)
    

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
  rospy.logwarn("staying in preparation for feeding for 30 seconds")
  rospy.sleep(9)
  rospy.logwarn("Going to start moving now!!!!!!!!!!!!!!!!!!!")
  t.inPreparationMode=False
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
  y_center = 0.195
  z_center = 0.45
  poses = [[ x_dist, y_center + 0.2 * np.sin(t),z_center + 0.2 * np.cos(t)] for t in times]

  for pose in poses+poses+poses:
    mesg = Point(pose[0], pose[1], pose[2])
    h = Header()
    h.stamp = rospy.Time.now()
    h.frame_id = "world"
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


