import tracker
import rospy

class State(Enum):
  WAIT_EMPTY = 1
  PICK_UP_FOOD = 2
  WAIT_FULL = 3
  MOVE_TO_MOUTH = 4
  WAIT_WHILE_BITE = 5
  MOVE_TO_PLATE = 6

class Spoon:
  def __init__(self):
    self.tracker = tracker.Tracker()
    self.state = State.WAIT_EMPTY

  def update_tracker_based_on_state(self):
    if self.state == State.WAIT_EMPTY or
       self.state == State.WAIT_FULL or
       self.state == State.WAIT_WHILE_BITE: 
      self.tracker.stop_moving()
    elif self.state == State.PICK_UP_FOOD:
      self.tracker.stop_moving()
      rospy.logwarn("PICKING UP FOOD") 
    elif self.state == State.MOVE_TO_MOUTH:
      self.tracker.start_updating_target_relative_to_mouth()
    elif self.state == State.MOVE_TO_PLATE: 
      self.tracker.start_tracking_fixed_target([0.3,-0.3,0.1])

if __name__=="__main__":
  # parse input arguments
  parser = argparse.ArgumentParser(description='node to track the position given by a rostopic')
  parser.add_argument('-s', '--sim', action='store_true',
                          help='simulation mode')
  args = parser.parse_args(rospy.myargv()[1:])
  # initialize the ros node 
  rospy.init_node('adapy_tracker', anonymous=True)
  s = Spoon()

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
  x_dist = 0.41676946
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


