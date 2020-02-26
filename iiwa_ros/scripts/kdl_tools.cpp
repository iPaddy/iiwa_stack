#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <csignal>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <iiwa_ros/state/joint_position.hpp>
#include <iostream>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>

void signalHandler(int /*unused*/) {
  ROS_INFO("Exiting...");
  exit(1);
}

void joint_read(){
  iiwa_ros::state::JointPosition jp_state;

  jp_state.init("iiwa");

  while (true){
    auto jp_pos = jp_state.getPosition();
    ROS_INFO_STREAM(
                    std::to_string(jp_pos.position.a1)
                    << " " << std::to_string(jp_pos.position.a2) << " " << std::to_string(jp_pos.position.a3)
                    << " " << std::to_string(jp_pos.position.a4) << " " << std::to_string(jp_pos.position.a5)
                    << " " << std::to_string(jp_pos.position.a6) << " " << std::to_string(jp_pos.position.a7)
                    << std::endl;);
    ros::Duration(0.1).sleep();
  }
}

void kdl_info(KDL::Tree tree){
  ROS_INFO_STREAM( "Number of Joints detected: "<< std::to_string(tree.getNrOfJoints()));
  ROS_INFO_STREAM( "Number of Segments detected: "<< std::to_string(tree.getNrOfSegments()));
}

int main(int argc, char **argv){
  // init node
  ros::init(argc, argv, "kdl_parser");

  // create kdl tree from parameter server for both robots if possible
  KDL::Tree tree;
  KDL::Tree tree2;
  ros::NodeHandle node;
  std::string robot_desc_string;
  node.param("iiwa/robot_description", robot_desc_string, std::string());
  if (!kdl_parser::treeFromString(robot_desc_string, tree)){
    ROS_ERROR("Failed to construct kdl tree");
    return false;
  }
  node.param("iiwa2/robot_description", robot_desc_string, std::string());
  if (!kdl_parser::treeFromString(robot_desc_string, tree2)){
    ROS_ERROR("Failed to construct kdl tree");
    return false;
  }

  // ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Signal handlers.
  signal(SIGTERM, signalHandler);
  signal(SIGINT, signalHandler);
  signal(SIGHUP, signalHandler);

  // output info on the kdl tree
  kdl_info(tree);
  kdl_info(tree2);

  // create solver based on kinematic tree
  KDL::TreeFkSolverPos_recursive fksolver = KDL::TreeFkSolverPos_recursive(tree);

  // Create joint array
  unsigned int nj = tree.getNrOfJoints();
  KDL::JntArray jointpositions = KDL::JntArray(nj);

  // Assign some values to the joint positions
  for(unsigned int i=0;i<nj;i++){
    float myinput;
    printf ("Enter the position of joint %i: ",i);
    scanf ("%e",&myinput);
    jointpositions(i)=(double)myinput;
    }

  // Create the frame that will contain the results
  KDL::Frame cartpos;

  // Calculate forward position kinematics
  bool kinematics_status;
  // todo get segmentName for last param
  kinematics_status = fksolver.JntToCart(jointpositions,cartpos,"iiwa_link_ee");
  if(kinematics_status>=0){
    std::cout << cartpos <<std::endl;
    printf("%s \n","Succes, thanks KDL!");
  }else{
    printf("%s \n","Error: could not calculate forward kinematics :(");
    }

  // TODO endeffector control with loaded kdl tree

  std::cerr << "Stopping spinner..." << std::endl;
  spinner.stop();

  std::cerr << "Bye!" << std::endl;
  return 0;
}
