#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
import numpy as np
import matplotlib.pyplot as plt
import tf
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from std_msgs.msg import MultiArrayDimension
from astar import AStar, StochOccupancyGrid2D, StochOccupancyGrid2D
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class Navigator:

    def __init__(self):
        rospy.init_node('navigator_project', anonymous=True)

        self.plan_resolution = 0.25
        self.plan_horizon = 15

        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin = [0,0]
        self.map_probs = []
        self.occupancy = None
        self.cur_nav_goal = [0,0,0]
        self.need_new_path = False

        self.nav_sp = None

        self.trans_listener = tf.TransformListener()
        self.cur_path = []

        rospy.Subscriber("map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("map_metadata", MapMetaData, self.map_md_callback)
        rospy.Subscriber("/turtlebot_controller/nav_goal", Float32MultiArray, self.nav_sp_callback)

        self.pose_sp_pub = rospy.Publisher('/turtlebot_controller/position_goal', Float32MultiArray, queue_size=10)
        self.nav_path_pub = rospy.Publisher('/turtlebot_controller/path_goal', Path, queue_size=10)
        self.path_validity = rospy.Publisher('/turtlebot_controller/path_validity', Int32MultiArray, queue_size = 10)
        self.nav_debug = rospy.Publisher('/turtlebot_controller/nav_debug', Float32MultiArray, queue_size = 10)

    def map_md_callback(self,msg):
        self.map_width = msg.width
        self.map_height = msg.height
        self.map_resolution = msg.resolution
        self.map_origin = (msg.origin.position.x,msg.origin.position.y)

    def map_callback(self,msg):
        self.map_probs = msg.data
        if self.map_width>0 and self.map_height>0 and len(self.map_probs)>0:
            self.occupancy = StochOccupancyGrid2D(self.map_resolution,
                                                  self.map_width,
                                                  self.map_height,
                                                  self.map_origin[0],
                                                  self.map_origin[1],
                                                  int(self.plan_resolution / self.map_resolution) * 2,
                                                  self.map_probs)

    def nav_sp_callback(self,msg):
        self.nav_sp = (msg.data[0],msg.data[1],msg.data[2])
        if np.all(self.cur_nav_goal != self.nav_sp):
            self.cur_nav_goal = self.nav_sp
            self.need_new_path = True
        self.send_pose_sp()
        
    def round_to_res(self, x):
        return self.plan_resolution * round(x/self.plan_resolution)

    def send_pose_sp(self):
        try:
            (robot_translation,robot_rotation) = self.trans_listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
            self.has_robot_location = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            robot_translation = (0,0,0)
            robot_rotation = (0,0,0,1)
            self.has_robot_location = False

        if self.occupancy and self.has_robot_location and self.nav_sp:
            state_min = (-int(round(self.plan_horizon)), -int(round(self.plan_horizon)))
            state_max = (int(round(self.plan_horizon)), int(round(self.plan_horizon)))
            x_init = (self.round_to_res(robot_translation[0]), self.round_to_res(robot_translation[1]))
            x_goal = (self.round_to_res(self.nav_sp[0]), self.round_to_res(self.nav_sp[1]))
            astar = AStar(state_min,state_max,x_init,x_goal,self.occupancy,self.plan_resolution)

            #debug = Float32MultiArray()
            #debug.layout.dim.append(MultiArrayDimension())
            #debug.layout.dim[0].label = "length"
            #debug.layout.dim[0].size = len(self.cur_path)
            #debug.layout.dim[0].stride = 1
            #debug.data = self.cur_path
            #self.nav_debug.publish(debug)

            rospy.loginfo("Computing navigation plan")
            if self.stillValid(astar) and not(self.need_new_path):
                flag = Int32MultiArray()
                flag.layout.dim.append(MultiArrayDimension())
                flag.layout.dim[0].label = "length"
                flag.layout.dim[0].size = 1
                flag.layout.dim[0].stride = 1
                flag.data = [1]
                self.path_validity.publish(flag)
            else:
                # reset because going to get a new path
                self.need_new_path = False
                flag = Int32MultiArray()
                flag.layout.dim.append(MultiArrayDimension())
                flag.layout.dim[0].label = "length"
                flag.layout.dim[0].size = 1
                flag.layout.dim[0].stride = 1
                flag.data = [0]
                self.path_validity.publish(flag)
                if astar.solve():
                    
                    # a naive path follower we could use
                    # pose_sp = (astar.path[1][0],astar.path[1][1],self.nav_sp[2])
                    # msg = Float32MultiArray()
                    # msg.data = pose_sp
                    # self.pose_sp_pub.publish(msg)
                    # astar.plot_path()
                    
                    path_msg = Path()
                    path_msg.header.frame_id = 'map'
                    self.cur_path = []
                    for state in astar.path:
                        pose_st = PoseStamped()
                        pose_st.pose.position.x = state[0]
                        pose_st.pose.position.y = state[1]
                        pose_st.header.frame_id = 'map'
                        path_msg.poses.append(pose_st)
                        self.cur_path.append([state[0],state[1]])
                    self.nav_path_pub.publish(path_msg)

                else:
                    rospy.logwarn("Could not find path")

    def stillValid(self, astarObj):
        for i in range(0,len(self.cur_path)):
            if not(astarObj.is_free(self.cur_path[i])):
                return False
        return True
      

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    nav = Navigator()
    nav.run()
