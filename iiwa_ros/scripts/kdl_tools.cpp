#include <ros/ros.h>
#include <stdio.h> // for user input
#include <iostream>
#include <csignal> // programm would not stop without
#include <array>

#include <kdl_parser/kdl_parser.hpp> //kdl from param server
#include <kdl/treefksolverpos_recursive.hpp> // ForwardKinematic solver
#include <kdl/chainfksolverpos_recursive.hpp> // ForwardKinematic solver
//#include <kdl/treeiksolverpos_recursive.hpp> // InverseKinematic solver
#include <kdl/treejnttojacsolver.hpp> // calculate jacobian solver
#include <kdl/frames.hpp> // save kinematic frame
#include <kdl/frames_io.hpp> // output frame to stream
#include <kdl/tree.hpp> // structure for saving the robot as a connected graph
#include <kdl/jntarray.hpp> // array for storing joint positions
#include <kdl/jntarrayvel.hpp> // array for storing joint velocities
#include <kdl/jacobian.hpp> // derivate for joint velocities
#include <kdl/treeiksolverpos_online.hpp>
#include <kdl/treeiksolvervel_wdls.hpp>

#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/ModelStates.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>

class IiwaKinematics{
public:
  IiwaKinematics(){
    // Subscribe to joint and puck positions
    iiwa_sub = nh.subscribe("iiwa2/joint_states", 1, &IiwaKinematics::jointStateCallback,this);
    puck_sub = nh.subscribe("gazebo/model_states", 1, &IiwaKinematics::puckPositionCallBack,this);
    // Publish for iiwa trajectory
    //pub = nh.advertise<control_msgs::FollowJointTrajectoryGoal>("iiwa/PositionJointInterface_trajectory_controller/command",1);
    pub = nh.advertise<trajectory_msgs::JointTrajectory>("iiwa2/PositionJointInterface_trajectory_controller/command",1);

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
    nh.param("iiwa2/robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, tree)){
      ROS_ERROR("Failed to construct kdl tree");
      return false;
    }
    nj = tree.getNrOfJoints();
    // Get chain from tree
    tree.getChain("iiwa_link_0", "striker_ee", chain);
    ROS_INFO_STREAM("number of joints in chain: " << chain.getNrOfJoints());
    ROS_INFO_STREAM("number of segments in chain: " << chain.getNrOfSegments());

    init_pos_iiwa = { 0.0, -1.53, 0.165 };
    KDL::Vector puck_pos(0.0,0.0,0.0);
    nj = tree.getNrOfJoints();
    q_cur = KDL::JntArray(nj);
    q_dot = KDL::JntArray(nj);
    q_min = KDL::JntArray(nj);
    q_max = KDL::JntArray(nj);
    q_dot_max = KDL::JntArray(nj);
    // output info on the kdl tree

    //set initial pos
    int ind = 0;
    point.trajectory.points.resize(1);
    point.trajectory.points[ind].positions.resize(7);
    point.trajectory.points[ind].positions[0] = 1.571;
    point.trajectory.points[ind].positions[1] = 1.05;
    point.trajectory.points[ind].positions[2] = 0.0;
    point.trajectory.points[ind].positions[3] = -1.571;
    point.trajectory.points[ind].positions[4] = 0.0;
    point.trajectory.points[ind].positions[5] = -0.3;
    point.trajectory.points[ind].positions[6] = -1.571;
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

    q_dot_max.data(0) = 10.0;
    q_dot_max.data(1) = 10.0;
    q_dot_max.data(2) = 10.0;
    q_dot_max.data(3) = 10.0;
    q_dot_max.data(4) = 10.0;
    q_dot_max.data(5) = 10.0;
    q_dot_max.data(6) = 10.0;
  }


  void puckPositionCallBack(const gazebo_msgs::ModelStates& msg){
    // find position of puck
    for(int i=0;i < msg.name.size();i++){
      if(!msg.name[i].compare("puck")){
        puck_pos(0)=msg.pose[i].position.x;
        puck_pos(1)=msg.pose[i].position.y;
        puck_pos(2)=msg.pose[i].position.z;
        ROS_INFO_STREAM("Puck position: x:" << puck_pos(0));
      }
    }
    subscribed_puck = true;
  }

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
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
  void ikSolver(){
      //iksolvervel = new KDL::TreeIkSolverVel_wdls(tree, "striker_ee");
      //iksolver = new KDL::TreeIkSolverPos_online(nj, "striker_ee", q_min, q_max, q_dot_max, 10.0, 10.0, fksolver, iksolvervel);
      //inv_kinematics_status = iksolver->CartToJnt(q_init,pos_des,q_aim);
      //if(inv_kinematics_status>=0){
      //std::cout << "iiwa needed joint position: " << pos_iiwa <<std::endl;
      //}else{
      //printf("%s \n","Error: could not calculate inverse kinematics :(");
      //}
    }
  void fkSolver(){
    // create solver based on kinematic tree
    // fksolver = new KDL::TreeFkSolverPos_recursive(tree);
    fksolver = new KDL::ChainFkSolverPos_recursive(chain);

    // todo get segmentName for last param
    //kinematics_status = fksolver.JntToCart(q_cur,cartpos,"iiwa_link_ee");
    //kinematics_status = fksolver->JntToCart(q_cur,cartpos,"striker_ee");
    kinematics_status = fksolver->JntToCart(q_cur,cartpos);
    if(kinematics_status>=0){
      // y axis should be inversed for iiwa - rotation 180 deg on z needed
      pos_iiwa =  init_pos_iiwa + cartpos.p;
      std::cout << "rotation : " << cartpos.M <<std::endl;
      std::cout << "translation : " << cartpos.p <<std::endl;
      std::cout << "iiwa ee position: " << pos_iiwa <<std::endl;
    }else{
      printf("%s \n","Error: could not calculate forward kinematics :(");
    }
  }
  void mainCallback(){
    pub.publish(point.trajectory);
    fkSolver();
  }

private:
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber iiwa_sub;
  ros::Subscriber puck_sub;
  KDL::Tree tree;
  KDL::Chain chain;
  std::string robot_desc_string;
  control_msgs::FollowJointTrajectoryGoal point;
  // Create joint array for initial and current positions and velocities
  unsigned int nj; // number of joints
  KDL::JntArray q_cur; //current joint position
  KDL::JntArray q_init; //initial joint position
  KDL::JntArray q_dot; // current joint velocity
  KDL::JntArray q_max; // maximum joint angles
  KDL::JntArray q_min; // minimal joint angles
  KDL::JntArray q_aim; // joint angles for reaching cart aim

  //KDL::TreeIkSolverPos_Online *iksolver;
  KDL::TreeIkSolverVel_wdls *iksolvervel;
  //KDL::TreeFkSolverPos_recursive *fksolver;
  KDL::ChainFkSolverPos_recursive *fksolver;
  KDL::JntArray q_dot_max; // maximum joint velocity

  // Create the frame that will contain the results
  KDL::Frame cartpos;
  KDL::Frame pos_des;

  KDL::Vector puck_pos;

  // endefector positions for both robots
  KDL::Vector pos_iiwa;
  // get the base position of iiwa as vector //TODO get position from gazebo (topic gazebo link state or Tree segment)
  KDL::Vector init_pos_iiwa;

  // Calculate forward position kinematics
  bool kinematics_status;
  // Calculate inverse position kinematics
  bool inv_kinematics_status;
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
