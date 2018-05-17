#!/usr/bin/env python
import numpy as np
import rospkg
import rospy

import transforms3d as t3d

from geometry_msgs.msg import Pose, Point, Quaternion

def load_position_list(filename):
  rospack = rospkg.RosPack()
  ada_tut_path = rospack.get_path("ada_tutorial")
  motion_data = np.loadtxt(ada_tut_path + "/data/" + filename, skiprows=1, delimiter=",")
  sampled_position_motion_data = get_sampled_pos_data(motion_data)
  return(sampled_position_motion_data)

def get_sampled_pos_data(rawData):
  datShape = rawData.shape
  sampleIndices = np.arange(0,datShape[0],100)
  # time, px, py, pz, ox, oy, oz
  datIndices = np.array([0,7,8,9,10,11,12])
  return(rawData[sampleIndices,:][:,datIndices]) 

# in tapo's data, ox is row[10], oy is row[11], and oz is row[12]
def get_quat(ox, oy, oz):
  quat = t3d.euler.euler2quat(oz, oy, ox, 'rzyx')
  return quat

if __name__=="__main__":
  rospy.init_node("simulate_spoon")
  pos_list = load_position_list("subject11_potato_salad/1.csv")
  starttime = rospy.Time.now().to_sec()
  lasttime = starttime + pos_list[-1,0] # in seconds
  curTime = rospy.Time.now().to_sec()
  pos_pub = rospy.Publisher("/target_pose", Pose, queue_size=10)
  r = rospy.Rate(10) # publish at max 10hz
  while curTime < lasttime:
    curRow = np.argmax(pos_list[:,0] + starttime > curTime) - 1
    curQuat = get_quat(pos_list[curRow,4], 
                       pos_list[curRow,5],
                       pos_list[curRow,6])
    pose = Pose(Point(pos_list[curRow,1],pos_list[curRow,2],pos_list[curRow,3]),
                Quaternion(curQuat[1],curQuat[2],curQuat[3],curQuat[0]))
    pos_pub.publish(pose)
    r.sleep()
    curTime = rospy.Time.now().to_sec()
