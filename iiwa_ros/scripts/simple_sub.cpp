#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

void jointStateCallBack(const sensor_msgs::JointState& msg){
  // getting position and velocity from current state readings
  auto points = msg.position;
  auto velocity = msg.velocity;
  ROS_INFO_STREAM("Joint Positions: " << points[0] << "," << points[1] << "," << points[2] << "," << points[3] << "," << points[4] << "," << points[5] << "," << points[6]);
  ROS_INFO_STREAM("Joint Velocities: " << velocity[0] << "," << velocity[1] << "," << velocity[2] << "," << velocity[3] << "," << velocity[4] << "," << velocity[5] << "," << velocity[6]);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "trajectory_sub");
  ros::NodeHandle n;
  ros::Rate r(100);
  ros::Subscriber sub;
  sub = n.subscribe("iiwa/joint_states", 1, jointStateCallBack);

  while(ros::ok()){
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
