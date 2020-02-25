#!/usr/bin/env python2

import rospy
import numpy as np
import actionlib
#from iiwa_msgs.msg import MoveToCartesianPoseAction, CartesianPose, MoveToCartesianPoseGoal
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# this should send a movement command to the robot in with the following structure:
#header:
#  seq: 4426
#  stamp:
#    secs: 0
#    nsecs:         0
#  frame_id: ''
#joint_names: [iiwa_joint_1, iiwa_joint_2, iiwa_joint_3, iiwa_joint_4, iiwa_joint_5, iiwa_joint_6,
#  iiwa_joint_7]
#points:
#  -
#    positions: [-2.93215314335, 0.0, 0.0, -0.0, -0.0, 0.0, 0.0]
#    velocities: []
#    accelerations: []
#    effort: []
#    time_from_start:
#      secs: 0
#      nsecs: 504999495
# to set a cartesian position a CartesianPose needs a PoseStamped postion
def set_pos():
    p = PoseStamped()
    p.pose.position.x =  0.0
    p.pose.position.y =  0.0
    p.pose.position.z = 0.0
    p.pose.orientation.x = 0.0
    p.pose.orientation.y = 0.0
    p.pose.orientation.z = 0.0
    p.pose.orientation.w = 1.0
    p.header.stamp = rospy.Time.now()

    return p

def set_joints():
    j = JointTrajectory()
    jp = JointTrajectoryPoint()
    jp.positions = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    pos = input("Input the robot arm joint angles:\n choose which joint [0-6]: ")
    jp.positions[pos] = input("input the angle in rad: ")
    jp.velocities = []
    jp.accelerations = []
    jp.effort = []
    jp.time_from_start.secs = 1
    j.points = [jp]
    j.header.frame_id = ''
    j.joint_names = ["iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"]
    return j

if __name__ == '__main__':
    pub = rospy.Publisher('/iiwa/PositionJointInterface_trajectory_controller/command', JointTrajectory, queue_size=10)
    pub2 = rospy.Publisher('/iiwa2/PositionJointInterface_trajectory_controller/command', JointTrajectory, queue_size=10)
    rospy.init_node('cartesian_control_client')
    rate = rospy.Rate(10) #Hz
    while not rospy.is_shutdown():
        p = set_pos()
        j = set_joints()
        pub.publish(j)
        pub2.publish(j)
        rate.sleep()

# Code dump
#sub = rospy.Subscriber('/iiwa/state/CartesianPose', CartesianPose, queue_size=1)
#pub = rospy.Publisher('/iiwa/state/CartesianPose', CartesianPose, queue_size=10)

# playing a client
    #client = actionlib.SimpleActionClient('cartesian_control', MoveToCartesianPoseAction)
    #client.wait_for_server()
    #goal = MoveToCartesianPoseGoal(set_pos())
    #client.send_goal_and_wait(rospy.Duration.from_sec(5.0))
