#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <std_msgs/String.h>
#include <iostream>
#include <trajectory_msgs/JointTrajectory.h>

int main(int argc, char **argv){
  ros::init(argc, argv , "trajectory_publisher");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<trajectory_msgs::JointTrajectory>("iiwa/PositionJointInterface_trajectory_controller/command", 1000);
  ros::Rate loop_rate(100);
  int count = 0;
  while(ros::ok()){
    trajectory_msgs::JointTrajectory traj;

    // set positions
    int ind = 0;
    traj.points.resize(1);
    traj.points[ind].positions.resize(7);
    traj.points[ind].positions[0] = 1.0;
    traj.points[ind].positions[1] = 1.0;
    traj.points[ind].positions[2] = 1.0;
    traj.points[ind].positions[3] = 0.0;
    traj.points[ind].positions[4] = 0.0;
    traj.points[ind].positions[5] = 0.0;
    traj.points[ind].positions[6] = 0.0;

    // set joint names
    traj.joint_names.push_back("iiwa_joint_1");
    traj.joint_names.push_back("iiwa_joint_2");
    traj.joint_names.push_back("iiwa_joint_3");
    traj.joint_names.push_back("iiwa_joint_4");
    traj.joint_names.push_back("iiwa_joint_5");
    traj.joint_names.push_back("iiwa_joint_6");
    traj.joint_names.push_back("iiwa_joint_7");

    traj.points[ind].time_from_start.sec = 1;

    pub.publish(traj);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;

  }

  return 0;

}
