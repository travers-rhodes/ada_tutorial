import numpy as np
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

# note that we're following the transform3d convention that the state representation
# is [x, y, z, qw, qx, qy, qz]
def get_transform_difference(curTransform, targetLoc, targetQuat):
  curLoc = curTransform[0:3,3]
  transDiff = targetLoc - curLoc
  curQuat = t3d.quaternions.mat2quat(curTransform[0:3,:][:,0:3])
  #print(curTransform[0:3,:][:,0:3])
  #print(curQuat)
  quatDiff = t3d.quaternions.qmult(t3d.quaternions.qinverse(curQuat), targetQuat)
  return(np.concatenate((transDiff, quatDiff)))

def mult_jacobian_with_direction(t3drotjac, quat_dir):
  # we do this out by hand to avoid any snafoos
  # we convert both to axis angle
  # then we dot product the axes
  # then we multiply by both angles
  jointDelta = np.zeros((t3drotjac.shape[1],))
  ax_quat, angle_quat = t3d.quaternions.quat2axangle(quat_dir)
  for i, joint in enumerate(np.transpose(t3drotjac)):
    ax_joint, angle_joint = t3d.quaternions.quat2axangle(joint)
    collinearity = np.dot(ax_quat, ax_joint)
    jointDelta[i] = collinearity*angle_quat*angle_joint
  return(jointDelta) 
  

if __name__=="__main__":
  import unittest
  def numpy_are_equal(mat1, mat2):
    epsilon = 0.00000001
    if np.all(np.abs(mat1 - mat2) < epsilon):
      return True
    else:
      print("The following matrices were not equal: %s \n %s"% (mat1, mat2))
      return False
  
  class TestMethods(unittest.TestCase):
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

    def test_get_transform_difference(self):
      self.assertTrue(numpy_are_equal(
        get_transform_difference(np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]),
                                 [1,1,1],
                                 [1,0,0,0]),
      	[1,1,1,1,0,0,0]))
      self.assertTrue(numpy_are_equal(
        get_transform_difference(np.array([[1,0,0,1],[0,1,0,2],[0,0,1,3],[0,0,0,1]]),
                                 [1,1,1],
                                 [1,0,0,0]),
      	[0,-1,-2,1,0,0,0]))
      self.assertTrue(numpy_are_equal(
        get_transform_difference(np.array([[0,-1,0,1],[1,0,0,2],[0,0,1,3],[0,0,0,1]]),
                                 [1,1,1],
                                 [1,0,0,0]),
      	[0,-1,-2,np.sqrt(0.5),0,0,-np.sqrt(0.5)]))
      #self.assertTrue(numpy_are_equal(
      #  get_transform_difference(np.array([[0,-1,0,0],[-1,0,0,0],[0,0,-1,0],[0,0,0,1]]),
      #                           [0,0,0],
      #                           [1,0,0,0]),
      #	[0,0,0,0,0,np.sqrt(0.5),-np.sqrt(0.5)]))

    def test_mult_jacobian_with_direction(self):
      self.assertTrue(numpy_are_equal(mult_jacobian_with_direction(np.transpose(np.array([[1,0,0,0],[1,0,0,0]])), np.array([1,0,0,0])), np.array([0,0])))
      self.assertTrue(numpy_are_equal(mult_jacobian_with_direction(np.transpose(np.array([[0.5,0.5,0,0],[1,0,0,0]])), np.array([0.5,0.5,0,0])), np.array([np.pi**2/4.0,0])))


  unittest.main()

