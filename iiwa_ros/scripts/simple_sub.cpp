#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/ModelStates.h>
#include <string>

void jointStateCallBack(const sensor_msgs::JointState& msg){
  // getting position and velocity from current state readings
  auto points = msg.position;
  auto velocity = msg.velocity;
  ROS_INFO_STREAM("Joint Positions: " << points[0] << "," << points[1] << "," << points[2] << "," << points[3] << "," << points[4] << "," << points[5] << "," << points[6]);
  ROS_INFO_STREAM("Joint Velocities: " << velocity[0] << "," << velocity[1] << "," << velocity[2] << "," << velocity[3] << "," << velocity[4] << "," << velocity[5] << "," << velocity[6]);
}

void puckPositionCallBack(const gazebo_msgs::ModelStates& msg){
  // find position of puck
  for(int i=0;i < msg.name.size();i++){
    if(!msg.name[i].compare("puck")) // inverse of my understanding of the compare function but it works
        ROS_INFO_STREAM("Puck position: x:" << msg.pose[i].position.x << " y: " <<
                        msg.pose[i].position.y << " z: " << msg.pose[i].position.z);
      //return msg.pose[i].position
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "trajectory_sub");
  ros::NodeHandle n;
  ros::Rate r(100);
  ros::Subscriber sub_joints;
  ros::Subscriber sub_puck;
  sub_joints = n.subscribe("iiwa/joint_states", 1, jointStateCallBack);
  sub_puck = n.subscribe("gazebo/model_states", 1, puckPositionCallBack);

  while(ros::ok()){
    ros::spin();
    r.sleep();
  }
  return 0;
}
