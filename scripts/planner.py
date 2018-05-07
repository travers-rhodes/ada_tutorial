#!/usr/bin/python
"""Shows how to set a physics engine and send torque commands to the robot
"""
import rospy
import argparse
from openravepy import *
import numpy as np
import time
import adapy
import transforms3d as t3d

from tf.msgs import tfMessage
from geometry_msgs.msgs import TransformStamped, Transform, Vectory3

rospy.init_node('adapy_tracker', anonymous=True)
parser = argparse.ArgumentParser(description='node to track the position given by a rostopic')
parser.add_argument('-s', '--sim', action='store_true',
                        help='simulation mode')
parser.add_argument('-v', '--viewer', nargs='?', const=True,
                        help='attach a viewer of the specified type')
parser.add_argument('--env-xml', type=str,
                        help='environment XML file; defaults to an empty environment')
parser.add_argument('--debug', action='store_true',
                        help='enable debug logging')
args = parser.parse_args()

class PositionCache:
  def __init__(self):
    self.target_listener = rospy.Subscriber("/feedbot/mouth_pose", tfMessage, self.target_listener_callback)
    # save off the latest mouth pose tf object seen by the vision system in a cached value
    # this object has
    # * Header header (int seq; time stamp; string frame_id)
    # * string child_frame_id
    # * geometry_msgs/Transform transform (Vectory3 translation; Quaternion rotation)
    self.latest_mouth_transform = None

  def target_listener_callback(self, mesg):
    self.latest_mouth_transform = mesg.transforms[0]

class Tracker:
  def __init__(self)
    RaveInitialize(True)
    misc.InitOpenRAVELogging()
    
    self.env, self.robot = adapy.initialize(
        sim=args.sim,
        attach_viewer=args.viewer,
        env_path=args.env_xml
    )
    
    self.manip = robot.SetActiveManipulator("Mico")
    self.manip_rob = interfaces.BaseManipulation(self.robot) # create the interface for basic manipulation programs

    # even though it's not obvious how we use this, we need to initialize the IKModel
    ikmodel = databases.inversekinematics.InverseKinematicsModel(self.robot,iktype=IkParameterization.Type.Transform6D)
    if not ikmodel.load():
      ikmodel.autogenerate()

    self.rot = self.generate_target_rotmat()

    self.pos_cache = PositionCache()

    # move to init position 
    self.move_to_target([[ 0.21676946, -0.2959789,   0.552686472]], timeoutSecs=0, allowRotation=True)

    # start tracking the actual mouth
    while not rospy.is_shutdown():
      self.track_target()

  def get_target_loc(self):
    if self.pos_cache is None:
      return None
    t =  self.pos_cache.latest_mouth_transform.transform.translation
    return np.array([t.x, t.y, t.z])

  def generate_target_rotmat(self):
    roty = t3d.quaternions.quat2mat([0,np.sqrt(2), 0, np.sqrt(2)])
    rotx = t3d.quaternions.quat2mat([np.sqrt(2),0, 0, np.sqrt(2)])
    rot = roty.dot(rotx)
    return rot

  def track_target(self, timeoutSecs, allowRotation=False):
    endLoc = self.get_target_loc() 
    if endLoc is None:
      rospy.sleep(timeoutSecs)
    self.move_to_target(endLoc, timeoutSecs, allowRotation=False)

  def move_to_target(self, endLoc, timeoutSecs, allowRotation=False)
    with self.env:
      Tgoal = np.concatenate((np.concatenate((self.rot, np.transpose(endLoc)), axis=1),[[0,0,0,1]]), axis=0)
      constrainttaskmatrix=np.eye(4)
      constraintmatrix = np.linalg.inv(self.manip.GetTransform())
      constrainterrorthresh = 0.005 # copied from tutorial
      constraintfreedoms = [0,0,0,0,0,0] # based on parsing the tutorial. Not sure why it's not 0,0,1,...
      if not allowRotation:
       # don't let any x,y rotations happen
       constraintfreedoms = [1,1,0,0,0,0] 
      manip_rob.MoveToHandPosition(matrices=[Tgoal],maxiter=3000,maxtries=1,seedik=40,
        constraintfreedoms=constraintfreedoms,
        constraintmatrix=constraintmatrix, 
        constrainttaskmatrix=constrainttaskmatrix,
        constrainterrorthresh=constrainterrorthresh,
        steplength=0.002)
    robot.WaitForController(timeoutSecs) # wait the timeout amount before choosing a new target

if __name__=="__main__":
  # start tracker running and start following locations
  t = Tracker()
  poses =[ 
                [[ 0.21676946, -0.2959789,   0.552686472]],
                [[ 0.21676946, -0.2959789,   0.52686472]],
                [[ 0.21676946, -0.2959789,   0.42686472]],
                [[ 0.31676946, -0.1959789,   0.42686472]],
                [[ 0.31676946, -0.3959789,   0.42686472]]]
  poses = []
  pub = rospy.Publisher('feedbot/mouth_pose', tfMessage, queue_size=10)
  h = std_msgs.msg.Header()
  h.stamp = rospy.Time.now()
  for pose in poses:
    mesg = tfMessage([TransformStamped(h, "dummy_frame", Transform(pose, None))])
    pub.publish(mesg)
    rospy.sleep(1)


