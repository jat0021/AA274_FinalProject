#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import String
from std_msgs.msg import Bool
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
    rospy.Subscriber('/turtlebot_controller/path_goal', Path, self.path_update)
    rospy.Subscriber('/turtlebot_controller/toSupervisor', String, self.updateFlag)
    self.debug = rospy.Publisher('/turtlebot_controller/supervisor_debug', String, queue_size=10)
    rospy.Subscriber('/turtlebot_controller/path_validity', Int32MultiArray, self.update_path_validity)
    self.tolerance = rospy.Publisher('turtlebot_controller/error_tolerance', String, queue_size = 10)
    self.success = rospy.Publisher('/success',Bool,queue_size = 10)
    self.trans_listener = tf.TransformListener()
    self.trans_broad = tf.TransformBroadcaster()
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)    # rviz "2D Nav Goal"
    self.has_tag_locs = False
    self.state = "manual driving"
    self.controlFlag = "not done"
    self.goal_idx = 1   # for tracking which tag number we're going to
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

    self.all_tag_numbers = range(8)

    # Subscriber to mission goals
    rospy.Subscriber('/mission', Int32MultiArray, self.updateMission)
    self.mission = []
    self.tags_found = [False,False,False,False,False,False,False,False,False,False]

    # Debug publisher with tag locations (float32 version)
    self.debug_pub = rospy.Publisher('/turtlebot_controller/supDebug', Float32MultiArray, queue_size = 10)

  def updateMission(self,data):
      self.mission = data.data

  def update_path_validity(self, flag):
      rospy.loginfo('recieved: %s', flag.data)
      if flag.data == 0:
        # false: need new path
        self.path_valid = flag.data
      # else: don't change flag until have gotten path back

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
        wp = self.waypoint_locations[tag_number].pose
        self.trans_broad.sendTransform((wp.position.x, wp.position.y, 0),
          (wp.orientation.x, wp.orientation.y, wp.orientation.z, wp.orientation.w),
          rospy.Time.now(),
          "waypoint_{0}".format(tag_number),
          "/map")

        # Publish debug message
        debugMsg = Float32MultiArray()
        debugMsg.data = np.array([tag_number, self.waypoint_locations[tag_number].pose.position.x, self.waypoint_locations[tag_number].pose.position.y])
        self.debug_pub.publish(debugMsg)
        self.tags_found[tag_number] = True

  def updateFlag(self, data):
    rospy.loginfo('received: %s', data.data)
    if data.data == "at dest":
      self.controlFlag = data.data
    # else don't update the flag, let state machine reset it

  def path_update(self, path):
    if not(self.path_valid):
      self.path_valid = 1
      # need a new path, so update
      n = len(path.poses)
      #self.debug.publish(str(n))
      if n == 1:
        ## put the one and only point on the path
        #x = path.poses[0].pose.position.x
        #y = path.poses[0].pose.position.y
        #self.path_locations = np.array([[x,y,0]])
        #self.path_index = 0
        # most likely lost the path, start navigating toward next known goal location?
        # set all this stuff for now and drive towards and hopefully a path will get planned
        self.path_locations = self.goal_locations[goal_idx,:]
        self.path_index = 0
        self.path_valid = 0
        self.control_flag = "not done"
        self.x_g = self.goal_locations[self.goal_idx,0]
        self.y_g = self.goal_locations[self.goal_idx,1]
        self.th_g = self.goal_locations[self.goal_idx,2]
        self.path_point = self.goal_locations[goal_idx,:]
        self.state = "normal"
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
        try:
          th = new_path_locs[-1,2]
        except:
          th = 0.0
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

  def loop(self):

      ######################################
	    # Your state machine logic goes here #
	    ######################################
    self.update_waypoints()
    if self.state == "manual driving":
      self.path_point = self.man_pose
      driveTo = Float32MultiArray()
      driveTo.layout.dim.append(MultiArrayDimension())
      driveTo.layout.dim[0].label = "length"
      driveTo.layout.dim[0].size = 3
      driveTo.layout.dim[0].stride = 1
      driveTo.data = self.path_point
      self.pos_sp_pub.publish(driveTo)
      all_tags = True
      for tag_num in self.mission:
        self.debug.publish(str(tag_num) + " " + str(self.tags_found))
        if not(self.tags_found[tag_num]):
          all_tags = False
      self.has_tag_locs = all_tags
      if self.has_tag_locs and (len(self.mission) != 0):
        self.state = "normal"
        self.controlFlag = "not done"
        self.goal_idx = 1
        for i in range(0,len(self.mission)):
          tag_num = self.mission[i]
          tag_x = self.waypoint_locations[tag_num].pose.position.x
          tag_y = self.waypoint_locations[tag_num].pose.position.y
          if i == 0:
            self.goal_locations = np.array([tag_x,tag_y,0])
          else:
            self.goal_locations = np.vstack((self.goal_locations,[tag_x,tag_y,0]))
        self.x_g = self.goal_locations[0,0]
        self.y_g = self.goal_locations[0,1]
        self.th_g = self.goal_locations[0,0]
        self.path_valid = 0

    #elif self.has_tag_location == False and self.controlFlag == "not done":
    #  self.state = "searching"
    elif self.controlFlag == "at dest":
      if self.end_of_path:
        if self.goal_idx == len(self.goal_locations):
          self.state = "stop"
          done_flag = Bool()
          done_flag.data = True
          self.success.publish(done_flag)
        else:
          # set next destination:
          self.control_flag = "not done"
          self.x_g = self.goal_locations[self.goal_idx,0]
          self.y_g = self.goal_locations[self.goal_idx,1]
          self.th_g = self.goal_locations[self.goal_idx,2]
          self.state = "normal"
          self.goal_idx = self.goal_idx + 1
          self.path_valid = 0
          self.end_of_path = False
      elif self.path_valid == 1:
        # set next path point
        if self.path_index == len(self.path_locations):
          self.path_index = self.path_index-1
        self.path_point = self.path_locations[self.path_index,:]
        self.path_index += 1
        if self.path_index == self.path_locations.shape[0]:
          self.end_of_path = True
          self.tolerance.publish("end")
        else:
          self.tolerance.publish("path")
          self.end_of_path = False
        self.controlFlag = "not done"
        self.state = "normal"

    else:
      self.state = "normal"

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
    #self.debug.publish(str(self.path_point) + str(self.goal_idx) + " " + str(self.path_index) + " "+str(self.path_locations.shape[0]))
    #self.debug.publish(str(self.tags_found))
    out = str(self.goal_locations)
    self.debug.publish(out)

    self.controlType.publish(self.state)

  def run(self):
    rate = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
      self.loop()
      rate.sleep()

if __name__ == '__main__':
  sup = Supervisor()
  sup.run()
