#include <ros/ros.h>
#include <stdio.h> // for user input
#include <iostream>
#include <csignal> // programm would not stop without
#include <array>
#include <unordered_map>
#include <math.h>

#include <chrono> // for timing debugging

#include <kdl_parser/kdl_parser.hpp> //kdl from param server
#include <kdl/chainfksolverpos_recursive.hpp> // ForwardKinematic solver
#include <kdl/chainiksolverpos_lma.hpp> // Inverse kinematic solver
#include <kdl/chainjnttojacsolver.hpp> // calculate jacobian solver
#include <kdl/chainiksolvervel_wdls.hpp> // calculate joint velocity from given cart vel
#include <kdl/frames.hpp> // save kinematic frame
#include <kdl/frames_io.hpp> // output frame to stream
#include <kdl/tree.hpp> // structure for saving the robot as a connected graph
#include <kdl/jntarray.hpp> // array for storing joint positions
#include <kdl/jntarrayvel.hpp> // array for storing joint velocities
#include <kdl/jacobian.hpp> // derivate for joint velocities

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <gazebo_msgs/ModelStates.h>

class IiwaKinematics{
public:
  IiwaKinematics(){
    // Subscribe to joint and puck positions
    iiwa_sub = nh.subscribe("iiwa/joint_states", 1, &IiwaKinematics::jointStateCallback,this);
    puck_sub = nh.subscribe("gazebo/model_states", 1, &IiwaKinematics::puckPositionCallBack,this);
    // Publish for iiwa joint angles
    pub = nh.advertise<std_msgs::Float64MultiArray>("iiwa/JointGroupPositionInterface_controller/command",1);

    // initialize joints as msg for sending to command the iiwa
    joints.layout.dim.resize(1);
    joints.layout.dim[0].size = 7;

    // Mapping joints to numbers for use in receiving the joint msgs
    j_map = {
            {0,"iiwa_joint_1"},
            {1,"iiwa_joint_2"},
            {2,"iiwa_joint_3"},
            {3,"iiwa_joint_4"},
            {4,"iiwa_joint_5"},
            {5,"iiwa_joint_6"},
            {6,"iiwa_joint_7"},
    };
    state_map = { {0, "init"}, {1, "reset"}, {2, "strike"} };
    count = 0;
}
  ~IiwaKinematics(){
  }

  bool initial(const std::string param, float rate){
    timestep = 1 / rate;
    prev_ee_pos = KDL::Vector(0.0,0.0,0.0);
    prev_puck_pos = KDL::Vector(0.0, 0.0, 0.0);

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

    KDL::Vector puck_pos(0.0,0.0,0.0);
    nj = tree.getNrOfJoints();
    q_cur = KDL::JntArray(nj);
    q_dot = KDL::JntArray(nj);
    q_aim = KDL::JntArray(nj);
    //q_min = KDL::JntArray(nj);
    //q_max = KDL::JntArray(nj);
    //q_dot_max = KDL::JntArray(nj);
    q_dot_aim = KDL::JntArray(nj);

    KDL::Vector vel_aim(10, 10, 0.0);
    KDL::Vector rot_(0.0, 0.0, 0.0);
    KDL::Twist vel_full_aim(vel_aim, rot_);

    //q_max.data(0) = -2.93; //q_min.data(0) = 2.93; //q_dot_max.data(0) = 100.0;
    //q_max.data(1) = -2.06; //q_min.data(1) = 2.06; //q_dot_max.data(1) = 100.0;
    //q_max.data(2) = -2.93; //q_min.data(2) = 2.93; //q_dot_max.data(2) = 100.0;
    //q_max.data(3) = -2.06; //q_min.data(3) = 2.06; //q_dot_max.data(3) = 100.0;
    //q_max.data(4) = -2.93; //q_min.data(4) = 2.93; //q_dot_max.data(4) = 100.0;
    //q_max.data(5) = -2.06; //q_min.data(5) = 2.06; //q_dot_max.data(5) = 100.0;
    //q_max.data(6) = -3.02; //q_min.data(6) = 3.02; //q_dot_max.data(6) = 100.0;

    // init of the robot position
    KDL::Vector goal_init_pos(0.0, -0.9, 0.165);
    pos_aim.p(0) = goal_init_pos[0];
    pos_aim.p(1) = goal_init_pos[1];
    pos_aim.p(2) = goal_init_pos[2];
    rot_aim.M = KDL::Rotation(0.0,0.0,1.0,
                              -0.707, -0.707, 0.0,
                              0.707,-0.707,0.0);

    // max reach with end effector in right rotation (0.0, -0.55, 0.17)
    KDL::Vector reach_pos(0.0,-0.55, 0.17);
    iiwa_pos = { 0.0,-1.53, 0.165 };
    max_reach_dist = (iiwa_pos - reach_pos).Norm();
    puck_to_goal_angle();
    following_traj = false;
    return true;
  }

  void reachedInitPos(){
    /**
        calculate distance between pos_aim and pos and if it is within threshold return true
        threshold chosen by hand to be 0.01
     */
    if(reached_init==true){
      return;
    }

    fkSolver();
    double dist_to_aim = (pos.p - pos_aim.p).Norm();
    //ROS_INFO_STREAM("distance to init position is: " << dist_to_aim);
    //ROS_INFO_STREAM("position pos : " << pos.p);
    //ROS_INFO_STREAM("position pos_aim : " << pos_aim.p);
    if(dist_to_aim < 0.01){
      ROS_INFO_STREAM("reached init position");
      reached_init = true;
      following_traj = false;
    }
    else{
      reached_init = false;
      following_traj = true;
    }
  }

  bool reachedAim(){
    // calculate distance between pos_aim and pos and if it is within threshold return true

    // updates the current position with fk based on joint angles
    fkSolver();

    double dist_to_aim = (pos.p - pos_aim.p ).Norm();
    //ROS_INFO_STREAM("distance to aim is: " << dist_to_aim);
    if(dist_to_aim < 0.01){
      //ROS_INFO_STREAM("reached aim");
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
        //ROS_INFO_STREAM("Puck position: y:" << puck_pos(1));
      }
    }
    subscribed_puck = true;
  }

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
    // write joint data to current joint angles and velocities
    q_cur.resize(msg->position.size());
    q_dot.resize(msg->velocity.size());
    for(int i=0; i<msg->position.size();i++){
      for(int j=0; j<joints.layout.dim[0].size; j++){
        // write that 0 -> joint_0
        if(j_map[j]==msg->name[i]){
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

  void puck_to_goal_angle(){
    /**
        calculates the angle to hit the puck for it to got towards the opponents goal
        angle is 0 if puck has to be hit straight forward (along y axis)
     */
    KDL::Vector goal_pos(0.0, 1.0, 0.175);
    KDL::Vector diff = goal_pos - puck_pos;
    //ROS_INFO_STREAM("Goal - puck: " << goal_pos - puck_pos );

    goal_angle = atan2(diff[1],diff[0]);
    //ROS_INFO_STREAM("Goal angle: " << goal_angle );

  }

  void direct_to_puck_traj(){
    //calculate intermediate points between two given points
    // given 1 second 50 points between the current ee position and the puck are calculated
    float time = 0.5; // secs
    int steps = int(time/timestep);
    // get distance from start to puck and divide by number of steps for each increase - linear interpolation
    //TODO substitute puck_pos with puck_pred
    //KDL::Vector diff = puck_pos - pos.p;
    KDL::Vector diff = puck_prediction() - pos.p;
    KDL::Vector start_pos = pos.p;
    float incr_x = diff[0] / steps;
    float incr_y = diff[1] / steps;
    //ROS_INFO_STREAM("starting trajectory pushing, with steps: " << steps);
    for(int i = 0; i < steps; i++){
      traj.push_back(KDL::Vector(start_pos[0] + incr_x*i, start_pos[1] + incr_y*i, 0.175 ));
      //ROS_INFO_STREAM("pushed trajectory point: x: " << start_pos[0] + incr_x * i << " y: " << start_pos[1] + incr_y * i);
    }
  }
  //void update_velocities(){
  //puck_vel = puck_pos - ;
  //ee_vel;
  //}

  void traj_to_puck(float duration, float velocity){
    // calculate intermediate points for trajectory from current position to puck
    // solving cubic polynomial with boundary conditions f(0),f(T),f_dot(0) and f_dot(T)
    // f(t) = at^3 + bt^2 + ct + d
    // f(0) - starting position => d = [x_0, y_0]
    // f_dot(0) - starting velocity => c = [x_dot_0, y_dot_0]
    // TODO get current velocity for striker and puck
    // TODO parametrize 50 for frequency
    int steps = int(duration/50);
  
  KDL::Vector diff = puck_pos - pos.p;
    ////traj.push_back(KDL::Vector(diff[0] + incr * i, diff[1] + incr * i, 0.175));
  }

  void follow_traj_point(){
    for (int i = 0; i < 3; i++)
    {
      pos_aim.p.data[i] = traj.front()[i];
    }
    traj.erase(traj.begin());
    updateAimPos();
  }

  void hitPuck(){
    // TODO write in a way that it takes for example 50 steps and after it is done sets following_traj to false
    // check if puck is in reachable area
    //if((puck_pos - iiwa_pos).Norm() < max_reach_dist){
    if((puck_prediction() - iiwa_pos).Norm() < max_reach_dist){
      // write hitting here
      //ROS_INFO_STREAM("Puck reachable: " << (puck_pos - iiwa_pos).Norm());
      //calc_traj(puck_pos);

      // follow basic interpolated trajectory
      if(traj.size() <= 0){
        direct_to_puck_traj();
      }
      follow_traj_point();
      //ROS_INFO_STREAM("number of trajectory points left: " << traj.size());

      // TODO use goal_angle for an advanced tactic, but basic hitting sufices for normal gameplay
      // TODO set velocity for hitting
    }
    else{
      //ROS_INFO_STREAM("Puck is not reachable with robot");
      following_traj = false;
      return;
    }
  }

  void calc_traj(KDL::Vector puck_pos){
    // solve trajectory by solving cubic hermite spline with boundary conditions of current position, end position, and the velocities at each point
  }

  void updateAimPos(){
    /**
        update the current aimed for position - overwriting the rotation to be fixed
        TODO check if rotational freedom of yaw can stay variable
     */
    pos_aim.M = rot_aim.M;

    // inverse kinematics solver giving the corresponding joint angles for a given cartesian position
    ikSolver();

    // update the joint positions with the calculated positions from inverse kinematics
    // conversion from KDL::JntArray to Float64MultiArray for sending ros messages
    joints.data.resize(7);
    joints.data = {q_aim.data[0], q_aim.data[1], q_aim.data[2], q_aim.data[3], q_aim.data[4], q_aim.data[5], q_aim.data[6]};

  }

  void ikSolver(){
    /**
       Inverse kinematics solver giving the matching joint angles for a cartesian point 
     */
    iksolver = new KDL::ChainIkSolverPos_LMA(chain);
    inv_kinematics_status = iksolver->CartToJnt(q_cur,pos_aim,q_aim);
    if(inv_kinematics_status>=0){
    }else{
      printf("%s \n","Error: could not calculate inverse kinematics :(");
    }
  }

  void fkSolver(){
    /**
       translating joint angles to cartesian positions given the kinematic chain
       writes position values to pos
     */
    fksolver = new KDL::ChainFkSolverPos_recursive(chain);

    kinematics_status = fksolver->JntToCart(q_cur,pos);
    if(kinematics_status>=0){
    }else{
      printf("%s \n","Error: could not calculate forward kinematics :(");
    }
  }

  void ikSolverVel(){
    iksolvervel = new KDL::ChainIkSolverVel_wdls(chain);

    vel_status = iksolvervel->CartToJnt(q_cur,vel_full_aim,q_dot_aim);
    if ( vel_status >=0){
    }else{
      printf("%s \n","Error: could not calculate joint velocities :(");
    }
  }
  void updateParams(){
    /**
       update the puck and endefector velocity values by storing previous position and doing simple derivation
       also storing the current positions as the previous one for the next call of this function
    */
    puck_vel = (puck_pos - prev_puck_pos)/timestep;
    ROS_INFO_STREAM("puck velocity: " << puck_vel);
    ee_vel = (pos.p - prev_ee_pos)/timestep;
    prev_ee_pos = pos.p;
    prev_puck_pos = puck_pos;
  }

  KDL::Vector puck_prediction(){
    //predict the position of the puck for a given amount of time
    KDL::Vector prediction = puck_pos + (timestep * puck_vel * 2.5);
    // x=[-0.43,0.43] y=[-0.9,0.9] z=0.11
    float x_max = 0.43;
    float x_min = -0.43;
    float y_max = 0.9;
    float y_min = -0.9;

    if(prediction[0] > x_max){
      prediction[0] = 2*x_max - prediction[0];
    }
    if(prediction[0] < x_min){
      prediction[0] = 2*x_min + prediction[0];
    }
    if(prediction[1] > y_max){
      prediction[1] = 2*y_max - prediction[1];
    }
    if(prediction[1] < y_min){
      prediction[1] = 2*y_min + prediction[1];
    }
    ROS_INFO_STREAM("current position: " << puck_pos);
    ROS_INFO_STREAM("prediction  puck: " << prediction);

    //TODO implement bounce - where are the borders, if it will meet one reverse the corresponding direction on impact
    return prediction;
  }

  void mainCallback(){
    // time events
    // used for debugging to see if a task like inverse kinematics takes too long
    using std::chrono::duration_cast;
    using std::chrono::nanoseconds;
    typedef std::chrono::high_resolution_clock clock;
    auto start = clock::now();

    // update velocities every 10th iteration
    if (!(count % 10)){
      updateParams();
    }
    count++;

    // update cartesian position given the joint angles
    fkSolver();

    // update the angle for the puck to go into the goal
    // oputputting radiant angle right now
    puck_to_goal_angle();
    //puck_prediction();

    // check if initialization has completed
    if(reached_init == false){
      reachedInitPos();
      updateAimPos();
    }
    // main loop for getting a new aim and updating accordingly
    if(reached_init==true && reachedAim() ){
      //ROS_INFO_STREAM("moving to next aim");
      // TODO some procedure here for getting the next command but has to be async for not blocking
      //assignAimPos();

      //updateAimPos();

    }

    //ROS_INFO_STREAM("currently following a trajectory?: " << following_traj);
    // if not currently following a trajectory start one
    if(not following_traj){
      // set trajectory and push to next aim
      //ROS_INFO_STREAM("Following trajectory");
      hitPuck();
    }
    // publish joint angles to robot
    pub.publish(joints);
    auto end = clock::now();

    // assuming a maximum duration of about 600000 ns leaving room for the 0.02 sec > 2,000,000ns 
    if (2000000 < duration_cast<nanoseconds>(end - start).count()){
        ROS_INFO_STREAM("taking to long for 50Hz");
      }
  }

private:
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber iiwa_sub, puck_sub;
  KDL::Tree tree;
  KDL::Chain chain;
  std::string robot_desc_string;
  std_msgs::Float64MultiArray joints;
  unsigned int nj; // number of joints
  std::unordered_map<int,std::string> j_map, state_map;
  KDL::JntArray q_cur, q_init, q_dot, q_max, q_min, q_aim; // all joint values from current, initial, velocity, min and max angles and aim
  KDL::ChainIkSolverPos_LMA *iksolver;
  KDL::ChainIkSolverVel_wdls *iksolvervel;
  KDL::ChainFkSolverPos_recursive *fksolver;
  KDL::JntArray q_dot_max, q_dot_aim; // maximum joint velocity
  KDL::Frame pos, pos_aim, rot_aim; // Create the frame that will contain the results
  KDL::Vector goal_init_pos, goal_pos, puck_pos, reach_pos, iiwa_pos, ee_vel, puck_vel; // endefector positions for both robots
  double max_reach_dist;
  bool kinematics_status, inv_kinematics_status, reached_init, vel_status;
  KDL::Twist vel_full_aim;
  KDL::Vector vel_aim;
  float goal_angle;
  std::vector<KDL::Vector> traj; // array for positions to go to
  uint count;
public:
  int steps;
  bool subscribed_puck,subscribed_joints,following_traj;
  KDL::Vector prev_puck_pos, prev_ee_pos;
  float timestep;

};

int main(int argc, char **argv){
  // init node
  ros::init(argc, argv, "kinematics");
  IiwaKinematics IiwaKinematics;

  // sets the frequency to 50Hz
  float frequency = 50.;
  ros::Rate r(frequency);
  std::string robotParam = "robot_description";

  if (IiwaKinematics.initial(robotParam, frequency))
  {
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
