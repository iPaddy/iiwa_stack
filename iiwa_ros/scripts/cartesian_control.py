#!/usr/bin/env python2

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def set_joints(pos_lst):
    j = JointTrajectory()
    jp = JointTrajectoryPoint()
    jp.positions = pos_lst
    pos = input("Input the robot arm joint angles:\n choose which joint [0-6]: ")
    jp.positions[pos] = input("input the angle in rad: ")
    jp.velocities = []
    jp.accelerations = []
    jp.effort = []
    #without this the iiwas wont move bc of timing issues
    jp.time_from_start.secs = 1
    j.points = [jp]
    j.header.frame_id = ''
    j.joint_names = ["iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"]
    return j,jp.positions

if __name__ == '__main__':
    pub = rospy.Publisher('/iiwa/PositionJointInterface_trajectory_controller/command', JointTrajectory, queue_size=10)
    pub2 = rospy.Publisher('/iiwa2/PositionJointInterface_trajectory_controller/command', JointTrajectory, queue_size=10)
    rospy.init_node('cartesian_control_client')
    rate = rospy.Rate(10) #Hz
    positions = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    while not rospy.is_shutdown():
        j,positions = set_joints(positions)
        pub.publish(j)
        pub2.publish(j)
        rate.sleep()

