#include <ros/ros.h>
#include <stdio.h> // for user input
#include <iostream>
#include <csignal> // programm would not stop without
#include <array>

#include <kdl_parser/kdl_parser.hpp> //kdl from param server
#include <kdl/chainfksolverpos_recursive.hpp> // ForwardKinematic solver
#include <kdl/chainiksolverpos_lma.hpp> // Inverse kinematic solver
#include <kdl/chainjnttojacsolver.hpp> // calculate jacobian solver
#include <kdl/frames.hpp> // save kinematic frame
#include <kdl/frames_io.hpp> // output frame to stream
#include <kdl/tree.hpp> // structure for saving the robot as a connected graph
#include <kdl/jntarray.hpp> // array for storing joint positions
#include <kdl/jntarrayvel.hpp> // array for storing joint velocities
#include <kdl/jacobian.hpp> // derivate for joint velocities

#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/ModelStates.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>

class IiwaKinematics{
public:
  IiwaKinematics(){
    // Subscribe to joint and puck positions
    iiwa_sub = nh.subscribe("iiwa/joint_states", 1, &IiwaKinematics::jointStateCallback,this);
    puck_sub = nh.subscribe("gazebo/model_states", 1, &IiwaKinematics::puckPositionCallBack,this);
    // Publish for iiwa trajectory
    //pub = nh.advertise<control_msgs::FollowJointTrajectoryGoal>("iiwa/PositionJointInterface_trajectory_controller/command",1);
    pub = nh.advertise<trajectory_msgs::JointTrajectory>("iiwa/PositionJointInterface_trajectory_controller/command",1);

    point.trajectory.header.stamp = ros::Time::now();

    point.trajectory.joint_names.push_back("iiwa_joint_1");
    point.trajectory.joint_names.push_back("iiwa_joint_2");
    point.trajectory.joint_names.push_back("iiwa_joint_3");
    point.trajectory.joint_names.push_back("iiwa_joint_4");
    point.trajectory.joint_names.push_back("iiwa_joint_5");
    point.trajectory.joint_names.push_back("iiwa_joint_6");
    point.trajectory.joint_names.push_back("iiwa_joint_7");
    //point.trajectory.joint_names.push_back("hand_joint");
    //point.trajectory.joint_names.push_back("hand_striker_joint");


}
  ~IiwaKinematics(){
  }

  bool initial(const std::string param){
    subscribed_joints = 0;
    subscribed_puck = 0;
    // create kdl tree from parameter server for both robots if possible
    nh.param("iiwa/robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, tree)){
      ROS_ERROR("Failed to construct kdl tree");
      return false;
    }
    nj = tree.getNrOfJoints();
    // Get chain from tree
    tree.getChain("world", "striker_ee", chain);
    ROS_INFO_STREAM("number of joints in chain: " << chain.getNrOfJoints());
    ROS_INFO_STREAM("number of segments in chain: " << chain.getNrOfSegments());

    // TODO make translation from this and add it to each pos
    init_iiwa_pos = {  -1.53, 0.0, 0.165 };
    KDL::Vector puck_pos(0.0,0.0,0.0);
    nj = tree.getNrOfJoints();
    q_cur = KDL::JntArray(nj);
    q_dot = KDL::JntArray(nj);
    q_aim = KDL::JntArray(nj);
    q_min = KDL::JntArray(nj);
    q_max = KDL::JntArray(nj);
    q_dot_max = KDL::JntArray(nj);
    // output info on the kdl tree

    //set initial pos this should define the rotation matrix for further usage
    int ind = 0;
    point.trajectory.points.resize(1);
    point.trajectory.points[ind].positions.resize(7);
    //point.trajectory.points[ind].positions[0] = 0.0;
    //point.trajectory.points[ind].positions[1] = 0.785;
    //point.trajectory.points[ind].positions[2] = 0.0;
    //point.trajectory.points[ind].positions[3] = -1.571;
    //point.trajectory.points[ind].positions[4] = 0.0;
    //point.trajectory.points[ind].positions[5] = 0.0;
    //point.trajectory.points[ind].positions[6] = 0.0;
    point.trajectory.points[ind].time_from_start.sec = 1;

    q_max.data(0) = -2.93;
    q_max.data(1) = -2.06;
    q_max.data(2) = -2.93;
    q_max.data(3) = -2.06;
    q_max.data(4) = -2.93;
    q_max.data(5) = -2.06;
    q_max.data(6) = -3.02;

    q_min.data(0) = 2.93;
    q_min.data(1) = 2.06;
    q_min.data(2) = 2.93;
    q_min.data(3) = 2.06;
    q_min.data(4) = 2.93;
    q_min.data(5) = 2.06;
    q_min.data(6) = 3.02;

    q_dot_max.data(0) = 100.0;
    q_dot_max.data(1) = 100.0;
    q_dot_max.data(2) = 100.0;
    q_dot_max.data(3) = 100.0;
    q_dot_max.data(4) = 100.0;
    q_dot_max.data(5) = 100.0;
    q_dot_max.data(6) = 100.0;

    pos_aim.p(0) = 0.0;
    pos_aim.p(1) = -0.9;
    pos_aim.p(2) = 0.18;
    rot_aim.M = KDL::Rotation(0.0,0.0,1.0,
                              -0.707, -0.707, 0.0,
                              0.707,-0.707,0.0);

  }
  void reachedInitPos(){
    // calculate distance between pos_aim and pos and if it is within threshold return true
    if(reached_init==true){
      return;
    }

    fkSolver();
    double dist_to_aim = (pos.p - pos_aim.p).Norm();
    ROS_INFO_STREAM("distance to init position is: " << dist_to_aim);
    ROS_INFO_STREAM("position pos : " << pos.p);
    ROS_INFO_STREAM("position pos_aim : " << pos_aim.p);
    ROS_INFO_STREAM("position iiwa_pos : " << iiwa_pos);
    if(dist_to_aim < 0.01){
      ROS_INFO_STREAM("reached init position");
      reached_init = true;
    }
    else{
      reached_init = false;
    }
  }

  bool reachedAim(){
    // calculate distance between pos_aim and pos and if it is within threshold return true

    // updates the current position with fk based on joint angles
    // TODO compare if getting values from gazebo could also work
    fkSolver();

    double dist_to_aim = (pos.p - pos_aim.p ).Norm();
    ROS_INFO_STREAM("distance to aim is: " << dist_to_aim);
    if(dist_to_aim < 0.01){
      ROS_INFO_STREAM("reached aim");
      return true;
    }
    else
      return false;
  }

  void puckPositionCallBack(const gazebo_msgs::ModelStates& msg){
    // find position of puck
    for(int i=0;i < msg.name.size();i++){
      if(!msg.name[i].compare("puck")){
        puck_pos(0)=msg.pose[i].position.x;
        puck_pos(1)=msg.pose[i].position.y;
        puck_pos(2)=msg.pose[i].position.z;
        //ROS_INFO_STREAM("Puck position: x:" << puck_pos(0));
      }
    }
    subscribed_puck = true;
  }

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
    // write joint data to current joint angles and velocities
    q_cur.resize(msg->position.size());
    q_dot.resize(msg->velocity.size());
    for(int i=0; i<msg->position.size();i++){
      for(int j=0; j<point.trajectory.joint_names.size(); j++){
        if(point.trajectory.joint_names[j]==msg->name[i]){
          q_cur.data(j)=msg->position[i];
          q_dot.data(j)=msg->velocity[i];
        }
      }
    }
    subscribed_joints = true;
  }
  void assignJoints(){
    // Assign some values to the joint positions
    for(unsigned int i=0;i<nj;i++){
      float myinput;
      printf ("Enter the position of joint %i: ",i);
      scanf ("%e",&myinput);
      q_cur.data(i)=(double)myinput;
    }
  }
  void assignAimPos(){
    // Assign some values to the joint positions
    for(unsigned int i=0;i<3;i++){
      float myinput;
      switch(i){
      case 0 : printf ("Enter the position of the desired x position : ");
        break;
      case 1 : printf ("Enter the position of the desired y position : ");
        break;
      case 2 : printf ("Enter the position of the desired z position : ");
        break;
      }
      scanf ("%e",&myinput);
      pos_aim.p(i)=(double)myinput;
    }
  }
  void updateAimPos(){
    pos_aim.M = rot_aim.M;

    ikSolver();

    int ind = 0;
    point.trajectory.points.resize(1);
    point.trajectory.points[ind].positions.resize(7);
    point.trajectory.points[ind].positions[0] = q_aim.data[0];
    point.trajectory.points[ind].positions[1] = q_aim.data[1];
    point.trajectory.points[ind].positions[2] = q_aim.data[2];
    point.trajectory.points[ind].positions[3] = q_aim.data[3];
    point.trajectory.points[ind].positions[4] = q_aim.data[4];
    point.trajectory.points[ind].positions[5] = q_aim.data[5];
    point.trajectory.points[ind].positions[6] = q_aim.data[6];
  }
  void ikSolver(){
      iksolver = new KDL::ChainIkSolverPos_LMA(chain);
      inv_kinematics_status = iksolver->CartToJnt(q_cur,pos_aim,q_aim);
      if(inv_kinematics_status>=0){
      }else{
        printf("%s \n","Error: could not calculate inverse kinematics :(");
      }
    }
  void fkSolver(){
    // create solver based on kinematic tree
    fksolver = new KDL::ChainFkSolverPos_recursive(chain);

    kinematics_status = fksolver->JntToCart(q_cur,pos);
    if(kinematics_status>=0){
    }else{
      printf("%s \n","Error: could not calculate forward kinematics :(");
    }
  }

  void mainCallback(){
    // publish joint angles to robot
    pub.publish(point.trajectory);
    // update cartesian position given the joint angles
    fkSolver();
    // check if initialization has completed
    if(reached_init == false){
      reachedInitPos();
      updateAimPos();
    }
    // main loop for getting a new aim and updating accordingly
    if(reached_init==true && reachedAim() ){
      ROS_INFO_STREAM("moving to next aim");
      assignAimPos();
      updateAimPos();
    }
  }

private:
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber iiwa_sub, puck_sub;
  KDL::Tree tree;
  KDL::Chain chain;
  std::string robot_desc_string;
  control_msgs::FollowJointTrajectoryGoal point;
  unsigned int nj; // number of joints
  KDL::JntArray q_cur, q_init, q_dot, q_max, q_min, q_aim; // all joint values from current, initial, velocity, min and max angles and aim
  KDL::ChainIkSolverPos_LMA *iksolver;
  KDL::ChainFkSolverPos_recursive *fksolver;
  KDL::JntArray q_dot_max; // maximum joint velocity
  KDL::Frame pos, pos_aim, rot_aim; // Create the frame that will contain the results
  KDL::Vector iiwa_pos, puck_pos; // endefector positions for both robots
  KDL::Vector init_iiwa_pos; // get the base position of iiwa as vector //TODO get position from gazebo (topic gazebo link state or Tree segment)
  bool kinematics_status, inv_kinematics_status, reached_init;

public:
  bool subscribed_puck,subscribed_joints;

};

int main(int argc, char **argv){
  // init node
  ros::init(argc, argv, "kinematics");
  IiwaKinematics IiwaKinematics;

  ros::Rate r(100);
  std::string robotParam = "robot_description";

  if(IiwaKinematics.initial(robotParam)){
  ROS_INFO_STREAM("initial success");
  }

  while(ros::ok()){
    ros::spinOnce();
    r.sleep();
    if(IiwaKinematics.subscribed_puck==1 && IiwaKinematics.subscribed_joints==1){
      IiwaKinematics.mainCallback();
    }
  }

  return 0;
}
