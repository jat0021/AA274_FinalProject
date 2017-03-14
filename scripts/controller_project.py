#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
import tf
import numpy as np
from scipy import linalg
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

class Controller:

    def __init__(self):
        rospy.init_node('controller_project', anonymous=True)
        #rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        ### problem 3 part 1 ###
        self.trans_listener = tf.TransformListener()

        rospy.Subscriber('/turtlebot_controller/position_goal', Float32MultiArray, self.updateGoal)
        rospy.Subscriber('/turtlebot_controller/control_type', String, self.updateControlType)
        self.toSupervisor = rospy.Publisher('/turtlebot_controller/toSupervisor', String, queue_size=10)
        self.debug = rospy.Publisher('/turtlebot_controller/debug', Float32MultiArray, queue_size=10)
        self.pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.x_g = 0.0
        self.y_g = 0.0
        self.th_g = 0.0
        self.flagState = "not done"
        self.controlMode = "normal"
        self.has_base_footprint = True

    def updateControlType(self, data):
        rospy.loginfo('received: %s', data.data)
        self.controlMode = data.data

    def updateGoal(self, data):
        rospy.loginfo('received: %s', data.data)
        # data in the format [x_g,y_g,th_g]
        self.x_g = data.data[0]
        self.y_g = data.data[1]
        self.th_g = data.data[2] 
        

#    def callback(self, data):
#        pose = data.pose[data.name.index("mobile_base")]
#        twist = data.twist[data.name.index("mobile_base")]
#        self.x = pose.position.x
#        self.y = pose.position.y
#        quaternion = (
#            pose.orientation.x,
#            pose.orientation.y,
#            pose.orientation.z,
#            pose.orientation.w)
#        euler = tf.transformations.euler_from_quaternion(quaternion)
#        self.theta = euler[2]

    def get_ctrl_output(self):
        # get robot_base location from teh mapping instead of gazebo model states
        try: 
            (translation,rotation) = self.trans_listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
            self.has_base_footprint = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException): 
            self.has_base_footprint = False
            translation = (0,0,0)
            rotation = (0,0,0,1)            
        if self.has_base_footprint == True:
            euler = tf.transformations.euler_from_quaternion(rotation)
            self.x = translation[0]
            self.y = translation[1]
            self.theta = euler[2]
        # else don't update current locations

        if self.controlMode == "searching":
            self.flagState = "not done"
            V = 0.0
            om = 0.5
        elif self.controlMode == "stop":
            V = 0.0
            om = 0.0
        else:
            # use self.x self.y and self.theta to compute the right control input here     
            distErr = 0.2  
#            angleErr = 0.3
            xErr = abs(self.x_g-self.x)
            yErr = abs(self.y_g-self.y)
            thErr = abs(self.th_g-self.theta)
            if xErr<distErr and yErr<distErr: # and thErr<angleErr:
                self.flagState = "at dest"
                V = 0.0
                om = 0.0
            else:
                self.flagState = "not done"
                rho = np.sqrt((self.x_g-self.x)*(self.x_g-self.x)+(self.y_g-self.y)*(self.y_g-self.y))
                alpha = np.arctan2((self.y_g-self.y),(self.x_g-self.x))-self.theta
                delta = np.arctan2((self.y_g-self.y),(self.x_g-self.x))-self.th_g
                k1 = .5
                k2 = .5
                k3 = .5

                #Define control inputs (V,om) - without saturation constraints
                V = k1*rho*np.cos(alpha)
                om = k2*alpha+k1*(np.sinc(alpha/np.pi)*np.cos(alpha))*(alpha+k3*delta)

                # Apply saturation limits
                V = np.sign(V)*min(0.5, np.abs(V))
                om = np.sign(om)*min(1, np.abs(om))

        cmd_x_dot = V # forward velocity
        cmd_theta_dot = om
        # end of what you need to modify
        cmd = Twist()
        cmd.linear.x = cmd_x_dot
        cmd.angular.z = cmd_theta_dot
        return cmd

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            ctrl_output = self.get_ctrl_output()
            self.pub.publish(ctrl_output)
            goal = Float32MultiArray()
            goal.layout.dim.append(MultiArrayDimension())
            goal.layout.dim[0].label="length"
            goal.layout.dim[0].size = 3
            goal.layout.dim[0].stride = 1
            goal.data = [self.x_g-self.x,self.y_g-self.y,self.th_g-self.theta]
            self.toSupervisor.publish(self.flagState)
            self.debug.publish(goal)
            rate.sleep()

if __name__ == '__main__':
    ctrl = Controller()
    ctrl.run()
