#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
import tf
import numpy as np
from scipy import linalg

class Controller:

    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        self.pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def callback(self, data):
        pose = data.pose[data.name.index("mobile_base")]
        twist = data.twist[data.name.index("mobile_base")]
        self.x = pose.position.x
        self.y = pose.position.y
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.theta = euler[2]

    def get_ctrl_output(self):
        # use self.x self.y and self.theta to compute the right control input here
        x_g = 1.0
        y_g = 1.0
        th_g = 0.0        
        rho = np.sqrt((x_g-self.x)*(x_g-self.x)+(y_g-self.y)*(y_g-self.y))
        alpha = np.arctan2((y_g-self.y),(x_g-self.x))-self.theta
        delta = np.arctan2((y_g-self.y),(x_g-self.x))-th_g
        k1 = .5
        k2 = .5
        k3 = .5

        #Define control inputs (V,om) - without saturation constraints
        V = k1*rho*np.cos(alpha)
        om = k2*alpha+k1*(np.sinc(alpha/np.pi)*np.cos(alpha))/alpha*(alpha+k3*delta)

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
            rate.sleep()

if __name__ == '__main__':
    ctrl = Controller()
    ctrl.run()
