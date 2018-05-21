import numpy as np
#from scipy.optimize import lsq_linear
import transforms3d as t3d

# given two 3d points curLoc and endLoc
# return the transformation associated with the frame
# with origin at curLoc and with the 
# z-axis pointing toward endLoc
# the other two axes are arbitrarily chosen
def get_motion_frame_matrix(curTransform, endLoc):
  curLoc = curTransform[0:3,3]
  curRot = curTransform[0:3,:][:,0:3]
  zDir = endLoc - curLoc
  xDir = perpendicular_vector(zDir)
  yDir = np.cross(zDir, xDir)
  zNorm = normalize(zDir) 
  xNorm = normalize(xDir)
  yNorm = normalize(yDir)
  rot = curRot.dot(np.concatenate(([xNorm], [yNorm], [zNorm]), axis=0).transpose())
  rot = np.concatenate(([xNorm], [yNorm], [zNorm]), axis=0).transpose()
  T = np.concatenate((np.concatenate((rot, np.zeros((3,1))), axis=1),[[0,0,0,1]]), axis=0)
  return(T)

# given a numpy vector
# return the numpy vector in the same direction of length 1
def normalize(v):
    norm = distance(v,np.zeros(v.shape))
    if norm == 0:
       raise ValueError('zero vector')
    return v / norm

def iszero(x):
  eps = 1e-15
  return np.absolute(x) < eps

# https://codereview.stackexchange.com/questions/43928/algorithm-to-get-an-arbitrary-perpendicular-vector
def perpendicular_vector(v):
  if iszero(v[0])  and iszero(v[1]):
      if iszero(v[2]):
          # v is Vector(0, 0, 0)
          raise ValueError('zero vector')
      # v is Vector(0, 0, v.z)
      return np.array([0, 1, 0])
  return np.array([-v[1], v[0], 0])

def distance(curLoc, endLoc):
  diff = endLoc - curLoc
  dist = np.sqrt(sum(diff**2))
  return dist

def quat_distance(curQuat, endQuat):
  diff = t3d.quaternions.qmult(t3d.quaternions.qinverse(curQuat), endQuat)
  return 1-(diff[0]**2)

# given the current transform and the target location and target orientation, 
# what is the translation and rotation we need to perform to move current transform to target
# PARAMS:
# * curTransform: A matrix that allows us to transform points written in EndEff frame to Base frame
# * targetLoc: the target xyz coordinate of the EndEff frame
# * targetQuat: the target quaternion rotation of the EndEff frame. [w,x,y,z]
# ***That is, if you convert targetLoc and targetQuat to a transformation matrix, that will give you
# ***a matrix that converts points written in the target frame to the base frame
# OUTPUT: 
# A vector following the transform3d convention that the state representation
# is [x, y, z, qw, qx, qy, qz]
# that gives the transformation (written in base frame coordinates) needed to apply to points in
# the target frame that returns points in the current frame.
# This can also be thought of as the transformation we need to add to the current frame in order to
# move that frame to the target frame.
def get_transform_difference_axis_angle(curTransform, targetLoc, targetQuat):
  curLoc = curTransform[0:3,3]
  transDiff = targetLoc - curLoc
  curMatTransform = curTransform[0:3,:][:,0:3]
  curQuat = t3d.quaternions.mat2quat(curMatTransform)
  quatDiff = t3d.quaternions.qmult(t3d.quaternions.qinverse(curQuat), targetQuat)
  #print("curquat is %s, targetQuat is %s, diffQuat is %s"%(curQuat, targetQuat, quatDiff))
  # we convert quatDiff to axis angle representation.
  # fun fact: because quatDiff is a rotation between cur and target frames, its axis-angle
  # representation is the same in both frames (axis of rotation doesn't move when rotation)
  diff_axis, diff_angle = t3d.quaternions.quat2axangle(quatDiff)
  #print("diffAxis, diffAngle is %s, %s"%(diff_axis, diff_angle))
  # now, we rotate diff_axis to the base frame and return
  diff_axis_base = curMatTransform.dot(diff_axis)
  #print(diff_axis_base)
  return((transDiff, diff_axis_base, diff_angle))

def convert_axis_angle_to_joint(angleAxesJoints, rotAxis, rotAngle):
  rotAxis = np.array(rotAxis)
  jointChange = np.transpose(angleAxesJoints).dot(rotAxis*rotAngle)
  return jointChange.transpose()
  
# Show the angle change necessary to approximately move the robot so that 
# it moves diffTrans and it rotates diffAngular
# PARAMS:
# ** jac: the linear translation jacobian (3xjoints)
# ** angVelJac: the list of axes in base frame coordinates around which each joint rotates (3xjoints)
# ** diffTrans: the desired translation of the end-effector in base coords
# ** diffAngular: the desired rotation of the end-effector (unit axis times rotation angle) 
def least_squares_step(jac, angVelJac, diffTrans, diffAngular, minConstraint=-np.inf, maxConstraint=np.inf):
  combinedJacs = np.concatenate((jac, angVelJac))
  #print(combinedJacs)
  combinedGoal = np.concatenate((diffTrans, diffAngular))
  #print(combinedGoal)
  #joints = lsq_linear(combinedJacs, combinedGoal,bounds=(minConstraint, maxConstraint))
  joints = np.linalg.lstsq(combinedJacs, combinedGoal)[0]
  #print(joints.success)
  #print(joints)
  return(joints)
  

if __name__=="__main__":
  import unittest

  def tuple_are_equal(tup1, tup2):
    allTrue = True
    for i,_ in enumerate(tup1):
      allTrue &= numpy_are_equal(tup1[i], tup2[i])
    return allTrue

  def numpy_are_equal(mat1, mat2):
    epsilon = 0.00000001
    if np.all(np.abs(mat1 - mat2) < epsilon):
      return True
    else:
      print("The following matrices were not equal: %s \n %s"% (mat1, mat2))
      return False
  
  class TestMethods(unittest.TestCase):
    def test_least_squares_step(self):
      self.assertTrue(numpy_are_equal(least_squares_step([[1,0],[0,1],[0,0]], [[1,0],[0,1],[0,0]],[1,1,0],[1,1,0]),
                                      [1,1]))
      self.assertTrue(numpy_are_equal(least_squares_step([[-1,0],[0,1],[0,0]], [[-1,0],[0,1],[0,0]],[1,1,0],[1,1,0]),
                                      [-1,1]))
      # Uncomment once we update numpy and support constrained least squares
      #self.assertTrue(numpy_are_equal(least_squares_step([[-1,0],[0,1],[0,0]], [[-1,0],[0,1],[0,0]],[1,1,0],[1,1,0], [0,0], [np.inf, np.inf]),
      #                                [0,1]))
      #self.assertTrue(numpy_are_equal(least_squares_step([[-1,0],[0,1],[0,0]], [[-1,0],[0,1],[0,0]],[1,1,0],[1,1,0], [-np.inf, -np.inf], [np.inf,0.5]),
      #                                [-1,0.5]))

    def test_get_motion_frame_matrix(self):
      self.assertTrue(numpy_are_equal(get_motion_frame_matrix(np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]),np.array([1,0,0])), 
                       np.array([[0,0,1,0],[1,0,0,0],[0,1,0,0],[0,0,0,1]])))
      self.assertTrue(numpy_are_equal(get_motion_frame_matrix(np.array([[1,0,0,0],[0,1,0,1],[0,0,1,1],[0,0,0,1]]),np.array([1,1,1])), 
                       np.array([[0,0,1,0],[1,0,0,0],[0,1,0,0],[0,0,0,1]])))
      self.assertTrue(numpy_are_equal(get_motion_frame_matrix(np.array([[1,0,0,1],[0,1,0,1],[0,0,1,0],[0,0,0,1]]),np.array([1,1,1])), 
                       np.array([[0,-1,0,0],[1,0,0,0],[0,0,1,0],[0,0,0,1]])))
      self.assertTrue(numpy_are_equal(get_motion_frame_matrix(np.array([[0,1,0,0],[-1,0,0,0],[0,0,1,0],[0,0,0,1]]),np.array([1,0,0])), 
                       np.array([[0,0,1,0],[1,0,0,0],[0,1,0,0],[0,0,0,1]])))

    def test_distance(self):
      self.assertAlmostEqual(distance(np.array([0,0,1]),np.array([0,0,0])),1)
      self.assertAlmostEqual(distance(np.array([1,2,3]),np.array([2,3,4])),np.sqrt(3))

    def test_get_transform_difference_axis_angle(self):
      self.assertTrue(tuple_are_equal(
        get_transform_difference_axis_angle(np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]),
                                 [1,1,1],
                                 [1,0,0,0]),
      	([1,1,1],[1,0,0],0)))
      self.assertTrue(tuple_are_equal(
        get_transform_difference_axis_angle(np.array([[1,0,0,1],[0,1,0,2],[0,0,1,3],[0,0,0,1]]),
                                 [1,1,1],
                                 [1,0,0,0]),
      	([0,-1,-2],[1,0,0],0)))
      # convert from rotated pi/2 around z back to origin, means we need to rotate negative pi/2
      self.assertTrue(tuple_are_equal(
        get_transform_difference_axis_angle(np.array([[0,-1,0,1],[1,0,0,2],[0,0,1,3],[0,0,0,1]]),
                                 [1,1,1],
                                 [1,0,0,0]),
      	([0,-1,-2],[0,0,-1],np.pi/2)))
      # convert from current starting position
      # to starting position rotated so that camera moves between hand and face
      self.assertTrue(tuple_are_equal(
        get_transform_difference_axis_angle(np.array([[0,0,1,0],[1,0,0,0],[0,1,0,0],[0,0,0,1]]),
                                 [0,0,0],
                                 [np.sqrt(0.5),0,np.sqrt(0.5),0]),
      	([0,0,0],[-1,0,0],np.pi/2)))

    def test_multiply_angVelJac_with_quat(self):
      self.assertTrue(numpy_are_equal(
        convert_axis_angle_to_joint(np.transpose(np.array([[1,0,0],[1,0,0]])), [1,0,0],0), 
        np.array([0,0])))
      self.assertTrue(numpy_are_equal(
        convert_axis_angle_to_joint(np.transpose(np.array([[np.sqrt(0.5),np.sqrt(0.5),0],[1,0,0]])), [1,0,0], np.pi/2),
        np.array([np.pi/2 * np.sqrt(0.5),np.pi/2])))
 
    def test_quat_distance(self):
      self.assertEqual(quat_distance([0,1,0,0], [0,1,0,0]), 0)
      self.assertEqual(quat_distance([0,1,0,0], [1,0,0,0]), 1)
      self.assertEqual(quat_distance([1/np.sqrt(2),1/np.sqrt(2),0,0], [1/np.sqrt(2),0,1/np.sqrt(2),0]), 3.0/4)


  unittest.main()

