#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import String
from nav_msgs.msg import Path
import tf
import numpy as np

class Supervisor:

  def __init__(self):
    rospy.init_node('supervisor_project', anonymous=True)
    self.pos_sp_pub = rospy.Publisher('/turtlebot_controller/position_goal', Float32MultiArray, queue_size=10)
    self.controlType = rospy.Publisher('/turtlebot_controller/control_type', String, queue_size=10)
    self.nav_goal = rospy.Publisher('/turtlebot_controller/nav_goal', Float32MultiArray, queue_size = 10)
    self.path_goal = rospy.Subscriber('/turtlebot_controller/path_goal', Path, self.path_update)
    rospy.Subscriber('/turtlebot_controller/toSupervisor', String, self.updateFlag)
    self.debug = rospy.Publisher('/turtlebot_controller/supervisor_debug', String, queue_size=10)
    self.trans_listener = tf.TransformListener()
    self.trans_broad = tf.TransformBroadcaster()
    self.has_tag_location = False
    self.x_g = 0.0
    self.y_g = 0.0
    self.th_g = 0.0
    self.state = "normal"
    self.controlFlag = "not done"
    self.goal_idx = 0   # for tracking which tag number we're going to
    # rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
    # use goal locations to store the tag locations in the order that we want to visit them for now
    self.goal_locations = np.array([[-2.0, 3.0, 0.0]])#np.array([[0,0,-np.pi/4.0],[-2.0, 3.0, 0.0]])
    self.path_locations = self.goal_locations
    self.path_index = 0
    self.path_point = self.goal_locations[0]
    self.end_of_path = False
    self.last_n = 0
    self.update_cnt = 0 # used so only run path_update every 10s

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

  def updateFlag(self, data):
    rospy.loginfo('received: %s', data.data)
    self.controlFlag = data.data

  def path_update(self, path):
    if self.update_cnt%100 == 0:
      n = len(path.poses)
      if n == 1:
        # put the one and only point on the path
        x = path.poses[0].pose.position.x
        y = path.poses[0].pose.position.y
        new_path_locs = np.array([[x,y,0]])
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

      if np.any(new_path_locs[:,0:2] != self.path_locations[self.path_index:,0:2]):
        # have a new path! takes into accound differing path lens too
        self.path_locations = new_path_locs
        self.path_index = 1
        self.path_point = self.path_locations[0,:]
        if self.path_index == self.path_locations.shape[0]:
          self.end_of_path = True
        else:
          self.end_of_path = False
      # else don't change the current path/path_index
    self.update_cnt += 1
  
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
    self.has_tag_location = True    # for 
    if self.has_tag_location == False and self.controlFlag == "not done": 
      self.state = "searching" 
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
    self.debug.publish(str(self.goal_idx))
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
