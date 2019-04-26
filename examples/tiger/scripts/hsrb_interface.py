#! /usr/bin/env python

import rospy
import actionlib
import numpy as np

from std_msgs.msg import *
from geometry_msgs.msg import *
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Point, Quaternion, PointStamped, Twist
# from hsrb_interface import Robot, exceptions, geometry
# import control_msgs.msg
# import controller_manager_msgs.srv
import trajectory_msgs.msg
from tiger.msg import TrackObsAction, TrackObsFeedback, TrackObsResult
# from sensor_msgs.msg import JointState
# from control_msgs.msg import JointTrajectoryControllerState
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
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
        self.direction_z=1
        self.gaze_theta_err=0.0
        self.desired_head_pan=0.0
        # Preparation to use robot functions

        # self.listener = tf.TransformListener()
	# self.listener.waitForTransform(_ORIGIN_TF,_BASE_TF, rospy.Time(), rospy.Duration(5.0))
        self.vel_pub = rospy.Publisher('/hsrb/command_velocity', geometry_msgs.msg.Twist,queue_size=10)
        # self.headPub = rospy.Publisher('/hsrb/head_trajectory_controller/command', JointTrajectory, queue_size=1)

        # jointstates_topic='hsrb/joint_states'
	# rospy.Subscriber(jointstates_topic, JointState, self.joint_state_Cb)

        robot_pose_topic='global_pose'
        rospy.Subscriber(robot_pose_topic, PoseStamped, self.robot_pose_Cb)

        self._as = actionlib.SimpleActionServer(self._action_name, TrackObsAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        # self.obs_cli = actionlib.SimpleActionClient('obscheck_action', tiger.msg.ObsCheckerAction)
        # self.obs_cli.wait_for_server()

    def process_target(self,target_point, gazetarget_point ):

        # target_point is represented w.r.t base_link
        goal_x = target_point.point.x
        goal_y = target_point.point.y
        goal_theta = target_point.point.z

        diff_x=goal_x;
        diff_y=goal_y;
        distance =0.0
        distance +=math.pow(diff_x,2)
        distance +=math.pow(diff_y,2)
        distance  =math.sqrt(distance)

        # coeff=(float)(1.0/split_size)
        if goal_theta>math.pi:
            goal_theta=goal_theta+2*math.pi
        elif goal_theta<(-1*math.pi):
            goal_theta=goal_theta+2*math.pi

        if goal_theta>0:
            self.direction_z=1
        else:
            self.direction_z=-1

        #gaze target processing
        self.gazetarget_yaw= gazetarget_point.point.z
        # self.gaze_theta_err =gaze_theta - self.robot_pose[3] 

        # if self.gaze_theta_err>1.0:
            # diff_angle=0.5;
        # elif self.gaze_theta_err<-1.0:
            # diff_angle=-0.5;

    def update_head_command(self):
        if self.yaw_err>0.45:
            self.desired_head_pan=self.robot_pose[3]+0.1*self.yaw_err
        else:
            self.desired_head_pan=self.robot_pose[3]+0.4*self.yaw_err

   
    def execute_cb(self, goal):
        rospy.loginfo("obs action called")
        obset = [];
        obset.append(0);
        obset.append(1);
        self.result_.success=True
        self._as.set_succeeded(self.result_)
        # self._as.publish_feedback(self.feedback_)

        self._as.set_succeeded()

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
    rospy.init_node('tiger_obs')
    server = hsr_interface('tiger_obs')
    rospy.loginfo("robot interface action server created")
    rospy.spin()
