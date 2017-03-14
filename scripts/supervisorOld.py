#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import String
import tf
import numpy as np

class Supervisor:

  def __init__(self):
    rospy.init_node('turtlebot_supervisor', anonymous=True)
    self.pos_sp_pub = rospy.Publisher('/turtlebot_controller/position_goal', Float32MultiArray, queue_size=10)
    self.controlType = rospy.Publisher('/turtlebot_controller/control_type', String, queue_size=10)
    rospy.Subscriber('/turtlebot_controller/toSupervisor', String, self.updateFlag)
    self.trans_listener = tf.TransformListener()
    self.trans_broad = tf.TransformBroadcaster()
    self.has_tag_location = False
    self.x_g = 0.0
    self.y_g = 0.0
    self.th_g = 0.0
    self.state = "stop"
    self.controlFlag = "not done"
    # rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)

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
    if self.has_tag_location == False and self.controlFlag == "not done": 
      self.state = "searching" 
    elif self.controlFlag == "at dest":
      self.state = "stop" 
    else:
      self.state = "normal"
      self.x_g = translation[0]
      self.y_g = translation[1]
      self.th_g = rotation[3]
    goal = Float32MultiArray()
    goal.layout.dim.append(MultiArrayDimension())
    goal.layout.dim[0].label="length"
    goal.layout.dim[0].size = 3
    goal.layout.dim[0].stride = 1
    goal.data = [self.x_g,self.y_g,self.th_g]
    self.pos_sp_pub.publish(goal)
    self.controlType.publish(self.state)

  def run(self):
    rate = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
      self.loop()
      rate.sleep()

if __name__ == '__main__':
  sup = Supervisor()
  sup.run()
