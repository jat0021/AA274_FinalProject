#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import String
from nav_msgs.msg import Path
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped

def pose_to_xyth(pose):
  th = tf.transformations.euler_from_quaternion((pose.orientation.x,
                                                 pose.orientation.y,
                                                 pose.orientation.z,
                                                 pose.orientation.w))[2]
  return [pose.position.x, pose.position.y, th]

class Supervisor:

  def __init__(self):
    rospy.init_node('supervisor_project', anonymous=True)
    self.pos_sp_pub = rospy.Publisher('/turtlebot_controller/position_goal', Float32MultiArray, queue_size=10)
    self.controlType = rospy.Publisher('/turtlebot_controller/control_type', String, queue_size=10)
    self.nav_goal = rospy.Publisher('/turtlebot_controller/nav_goal', Float32MultiArray, queue_size = 10)
    self.path_goal = rospy.Subscriber('/turtlebot_controller/path_goal', Path, self.path_update)
    rospy.Subscriber('/turtlebot_controller/toSupervisor', String, self.updateFlag)
    self.debug = rospy.Publisher('/turtlebot_controller/supervisor_debug', String, queue_size=10)
    rospy.Subscriber('/turtlebot_controller/path_validity', Int32MultiArray, self.update_path_validity)
    self.trans_listener = tf.TransformListener()
    self.trans_broad = tf.TransformBroadcaster()
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)    # rviz "2D Nav Goal"
    self.has_tag_location = False
    self.state = "normal"
    self.controlFlag = "not done"
    self.goal_idx = 1   # for tracking which tag number we're going to
    # rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
    # use goal locations to store the tag locations in the order that we want to visit them for now
    self.goal_locations = np.array([[0.0, 3.0, 0.0]])#,[3,1,0]])
    self.path_locations = self.goal_locations
    self.x_g = self.goal_locations[0,0]
    self.y_g = self.goal_locations[0,1]
    self.th_g = self.goal_locations[0,2]
    self.path_index = 0
    self.path_point = self.goal_locations[0]
    self.end_of_path = False
    #self.last_n = 0
    #self.update_cnt = 0 # used so only run path_update every 10s
    self.man_pose = [0,0,0]
    self.path_valid = False;

    # Fiducial location initialization
    self.waypoint_locations = {}    # dictionary that caches the most updated locations of each mission waypoint
    self.waypoint_offset = PoseStamped()
    self.waypoint_offset.pose.position.z = .4    # waypoint is located 40cm in front of the AprilTag, facing it
    quat = tf.transformations.quaternion_from_euler(0., np.pi/2, np.pi/2)
    self.waypoint_offset.pose.orientation.x = quat[0]
    self.waypoint_offset.pose.orientation.y = quat[1]
    self.waypoint_offset.pose.orientation.z = quat[2]
    self.waypoint_offset.pose.orientation.w = quat[3]

    # Subscriber to mission goals
    rospy.Subscriber('/mission', Int32MultiArray, self.updateMission)
    self.mission = []

    # Debug publisher with tag locations (float32 version)
    self.debug_pub = rospy.Publisher('/turtlebot_controller/supDebug', Float32MultiArray, queue_size = 10)

#  def gazebo_callback(self, data):
#    pose = data.pose[data.name.index("mobile_base")]
#    quaternion = (pose.orientation.x,
#                  pose.orientation.y,
#                  pose.orientation.z,
#                  pose.orientation.w)
#    self.trans_broad.sendTransform((pose.position.x,pose.position.y,0),
#                    quaternion,
#                    rospy.Time.now(),
#                    "mobile_base",
#                    "world")

  def updateMission(self,data):
      self.mission = data.data

  def update_path_validity(self, flag):
      rospy.loginfo('recieved: %s', flag.data)
      self.path_valid = flag.data

  def rviz_goal_callback(self, msg):
      self.man_pose = pose_to_xyth(msg.pose)    # example usage of the function pose_to_xyth (defined above)
      # this callback does nothing... yet!

  def update_waypoints(self):
    for tag_number in self.mission:
      try:
        self.waypoint_offset.header.frame_id = "/tag_{0}".format(tag_number)
        self.waypoint_locations[tag_number] = self.trans_listener.transformPose("/map", self.waypoint_offset)

      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass

      if tag_number in self.waypoint_locations:
        # Publish debug message
        debugMsg = Float32MultiArray()
        debugMsg.data = np.array([tag_number, self.waypoint_locations[tag_number].pose.position.x, self.waypoint_locations[tag_number].pose.position.y])
        self.debug_pub.publish(debugMsg)


  def updateFlag(self, data):
    rospy.loginfo('received: %s', data.data)
    self.controlFlag = data.data

  def path_update(self, path):
    if not(self.path_valid):
      # need a new path, so update
      n = len(path.poses)
      self.debug.publish(str(n))
      if n == 1:
        # put the one and only point on the path
        x = path.poses[0].pose.position.x
        y = path.poses[0].pose.position.y
        self.path_locations = np.array([[x,y,0]])
        self.path_index = 0
      else:
        for i in range(0,n-1):
          x = path.poses[i].pose.position.x
          x2 = path.poses[i + 1].pose.position.x
          y = path.poses[i].pose.position.y
          y2 = path.poses[i + 1].pose.position.y
          th = np.arctan2((y2 - y), (x2 - x))
          if i == 0:
            new_path_locs = np.array([x,y,th])
          else:
            new_path_locs = np.vstack((new_path_locs, [x,y,th]))
        # add last point:
        x = path.poses[-1].pose.position.x
        y = path.poses[-1].pose.position.y
        th = new_path_locs[-1,2]
        new_path_locs = np.vstack((new_path_locs, [x,y,th]))
        self.path_locations = new_path_locs
        if n>1:
          self.path_index = 2
          self.path_point = new_path_locs[1,:]
          self.end_of_path = False
        else:
          self.path_index = 1
          self.path_point = new_path_locs[0,:]
          self.end_of_path = True

      

#  def path_update(self, path):
#    if self.update_cnt%50 == 0:
#      n = len(path.poses)
#      if n == 1:
#        # put the one and only point on the path
#        x = path.poses[0].pose.position.x
#        y = path.poses[0].pose.position.y
#        new_path_locs = np.array([[x,y,0]])
#        self.path_index = 0
#      else:
#        for i in range(0,n-1):
#          x = path.poses[i].pose.position.x
#          x2 = path.poses[i + 1].pose.position.x
#          y = path.poses[i].pose.position.y
#          y2 = path.poses[i + 1].pose.position.y
#          th = np.arctan2((y2 - y), (x2 - x))
#          if i == 0:
#            new_path_locs = np.array([x,y,th])
#          else:
#            new_path_locs = np.vstack((new_path_locs, [x,y,th]))
#        # add last point:
#        x = path.poses[-1].pose.position.x
#        y = path.poses[-1].pose.position.y
#        th = new_path_locs[-1,2]
#        new_path_locs = np.vstack((new_path_locs, [x,y,th]))
#
#
#      if np.any(new_path_locs[:,0:2] != self.path_locations[self.path_index:,0:2]):
#        # have a new path! takes into accound differing path lens too
#        self.path_locations = new_path_locs
#        self.path_index = 1
#        self.path_point = self.path_locations[0,:]
#        if self.path_index == self.path_locations.shape[0]:
#          self.end_of_path = True
#        else:
#          self.end_of_path = False
#      # else don't change the current path/path_index
#    self.update_cnt += 1
  
#    if n != self.last_n:
#      self.path_index = 1
#    self.last_n = n
#    
#    if n>2:
#      obtained_point = False
#      
#      while not obtained_point:
#        if self.path_index >= n-1:
#          self.path_index -= 1
#        else:
#          obtained_point = True
#      
#      x = path.poses[self.path_index].pose.position.x
#      x2 = path.poses[self.path_index + 1].pose.position.x
#      y = path.poses[self.path_index].pose.position.y
#      y2 = path.poses[self.path_index + 1].pose.position.y
#      th = np.arctan2((y2 - y), (x2 - x))
#      
#    elif n == 2:
#      x = path.poses[1].pose.position.x
#      x1 = path.poses[0].pose.position.x
#      y = path.poses[1].pose.position.y
#      y1 = path.poses[0].pose.position.y
#      th = np.arctan2((y - y1), (x - x1))
#      self.path_index = 1
#      self.end_of_path = True
#
#    else:
#      x = self.path_point[0]
#      y = self.path_point[1]
#      th = self.path_point[2]
#      self.path_index = 0
#      self.end_of_path = True
#      
#    self.path_point = [x, y, th]
  
  def loop(self):
    try:
      # tf knows where the tag is
      # (translation,rotation) = self.trans_listener.lookupTransform("/world", "/tag_0", rospy.Time(0))
      (translation,rotation) = self.trans_listener.lookupTransform("/map", "/tag_0", rospy.Time(0))
      self.has_tag_location = True
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      # tf doesn't know where the tag is
      translation = (0,0,0)
      rotation = (0,0,0,1)
      self.has_tag_location = False
  
      ######################################
	    # Your state machine logic goes here #
      # spin in place for now and look for a tag???
	    ######################################
      # code to publish location goal here?
    self.has_tag_location = True    # for now
    if self.state == "manual driving":
      self.update_waypoints()
      self.path_point = self.man_pose
      driveTo = Float32MultiArray()
      driveTo.layout.dim.append(MultiArrayDimension())
      driveTo.layout.dim[0].label = "length"
      driveTo.layout.dim[0].size = 3
      driveTo.layout.dim[0].stride = 1
      driveTo.data = self.path_point
      self.pos_sp_pub.publish(driveTo)

    #elif self.has_tag_location == False and self.controlFlag == "not done": 
    #  self.state = "searching" 
    elif self.controlFlag == "at dest":
      if self.end_of_path:
        if self.goal_idx == len(self.goal_locations):
          self.state = "stop"
        else:
          # set next destination:
          self.x_g = self.goal_locations[self.goal_idx,0]
          self.y_g = self.goal_locations[self.goal_idx,1]
          self.th_g = self.goal_locations[self.goal_idx,2]
          self.state = "normal" 
          self.goal_idx = self.goal_idx + 1
      else:
        # set next path point
        self.path_point = self.path_locations[self.path_index,:]
        self.path_index += 1
        if self.path_index == self.path_locations.shape[0]:
          self.end_of_path = True
        else:
          self.end_of_path = False
        self.controlFlag = "not done"
        self.state = "normal"

    else:
      self.state = "normal"
      #self.x_g = translation[0]
      #self.y_g = translation[1]
      #self.th_g = rotation[3]

    # dummy supervisor constant goal:
    
    navTo = Float32MultiArray()
    navTo.layout.dim.append(MultiArrayDimension())
    navTo.layout.dim[0].label = "length"
    navTo.layout.dim[0].size = 3
    navTo.layout.dim[0].stride = 1
    navTo.data = [self.x_g,self.y_g,self.th_g]
    self.nav_goal.publish(navTo)
    
    driveTo = Float32MultiArray()
    driveTo.layout.dim.append(MultiArrayDimension())
    driveTo.layout.dim[0].label = "length"
    driveTo.layout.dim[0].size = 3
    driveTo.layout.dim[0].stride = 1
    driveTo.data = self.path_point
    self.pos_sp_pub.publish(driveTo)
    # debug:
    #self.debug.publish(str(self.goal_idx))
    # self.debug.publish(str(self.path_point) + str(self.path_index) + " "+str(self.path_locations.shape[0]))

    self.controlType.publish(self.state)
    
  def run(self):
    rate = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
      self.loop()
      rate.sleep()

if __name__ == '__main__':
  sup = Supervisor()
  sup.run()
