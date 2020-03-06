#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
//#include <gazebo_msgs/LinkStates.h>
//#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

class RobotArm{
private:
  control_msgs::FollowJointTrajectoryGoal point;

public:
  TrajClient* traj_client;
  RobotArm(){
    traj_client = new TrajClient("iiwa/PositionJointInterface_trajectory_controller/follow_joint_trajectory",true);
    while(!traj_client->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  }

  ~RobotArm(){
    delete traj_client;
  }

  void startTrajectory(control_msgs::FollowJointTrajectoryGoal& point){

    point.trajectory.header.stamp = ros::Time::now();
    point.trajectory.points[0].time_from_start = ros::Duration(0.1);

    traj_client->sendGoal(point);
  }

  void followTrajectoryCallBack(const control_msgs::FollowJointTrajectoryGoal& msg){
    point=msg;
    startTrajectory(point);
  }
  void jointStateCallBack(const sensor_msgs::JointState& msg){
    // getting position and velocity from current state readings
    auto points = msg.position;
    auto velocity = msg.velocity;
    // TODO do sth with those
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "trajectory_pub");
  ros::NodeHandle n;
  ros::Rate r(100);
  ros::Subscriber sub;
  RobotArm arm;
  sub = n.subscribe("iiwa/joint_states", 1, &RobotArm::jointStateCallBack, &arm);

  while(ros::ok() && arm.traj_client->isServerConnected()){
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
