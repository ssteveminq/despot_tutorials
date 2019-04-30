#! /usr/bin/env python

import rospy
import actionlib
import numpy as np
from numpy.random import uniform

from std_msgs.msg import *
from geometry_msgs.msg import *
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Point, Quaternion, PointStamped, Twist
# from hsrb_interface import Robot, exceptions, geometry
import trajectory_msgs.msg
from tiger.msg import TrackObsAction, TrackObsFeedback, TrackObsResult
import tf
from tf import TransformListener
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

_ORIGIN_TF ='map'
_BASE_TF = 'base_link'


class hsr_interface(object):

    def __init__(self, name):
        # Init actionserver
        self._action_name = name
        # self.robot = robot
        self.robot_pose=np.zeros((5,1))
        #x,y,theta
        self.vel_cmd = Twist()
        self.target_point = PointStamped()
        self.gazetarget_point = PointStamped()
        self.target_pose = Point()
        self.feedback_ = TrackObsFeedback()
        self.result_= TrackObsResult()
        self.target_yaw=0.0
        self.noise_probability=0.15
        self.object_position =1;
        # Preparation to use robot functions

        self.vel_pub = rospy.Publisher('/hsrb/command_velocity', geometry_msgs.msg.Twist,queue_size=10)

        robot_pose_topic='global_pose'
        rospy.Subscriber(robot_pose_topic, PoseStamped, self.robot_pose_Cb)

        self._as = actionlib.SimpleActionServer(self._action_name, TrackObsAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        rospy.loginfo("action_server obs action called")
        # print goal
        print("----------------------")
        self.object_position = 0
        if goal.action == 0 or goal.action==1: #search & move_see action
            self.object_position =1
            obs = 2
        else:
            reward = -1
            coin = np.random.uniform()
            # print coin
            if (coin <= 1.0-self.noise_probability):
                obs = self.object_position
            else:
                obs =1-self.object_position 
            
        self.result_.state = self.object_position
        self.result_.observations = obs
        self.result_.reward = 0
        self.result_.success=True
        self._as.set_succeeded(self.result_)
        # self._as.publish_feedback(self.feedback_)


    # def joint_state_Cb(self, msg):
        # self.robot_pose[3]=msg.position[9]
        # self.robot_pose[4]=msg.position[10]


    def robot_pose_Cb(self, msg):

        self.robot_pose[0]=msg.pose.position.x
        self.robot_pose[1]=msg.pose.position.y
        robot_orientation=msg.pose.orientation

        #get yaw angle from quarternion
        orientation_list=[robot_orientation.x, robot_orientation.y, robot_orientation.z,robot_orientation.w]
        roll,pitch,yaw=euler_from_quaternion(orientation_list)
        self.robot_pose[2]=yaw


    def send_vel_command(self):
        self.vel_pub.publish(self.vel_cmd)


if __name__ == '__main__':
    rospy.init_node('search_obs')
    server = hsr_interface('search_obs')
    rospy.loginfo("robot interface action server created")
    rospy.spin()
