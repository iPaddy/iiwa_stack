#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <std_msgs/String.h>
#include <iostream>
#include <ctime>
#include <gazebo_msgs/ModelState.h>

int main(int argc, char **argv){
  ros::init(argc, argv , "reset_puck");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000);
  ros::Rate loop_rate(100);
  int count = 0;
  while(ros::ok() && count <= 2){
    gazebo_msgs::ModelState model;

    model.model_name = "puck";
    // sets the puck to a random position on the table within the play area
    // full table is x=[-0.43,0.43] y=[-0.9,0.9] z=0.11
    std::srand(std::time(0));
    //model.pose.position.x = -0.43 + ((float(rand())/float(RAND_MAX)) * 0.86);
    //model.pose.position.y = -0.9  + ((float(rand())/float(RAND_MAX)) * 1.8);
    model.pose.position.x = -0.43 + ((float(rand())/float(RAND_MAX)) * 0.86);
    model.pose.position.y = -0.9  + ((float(rand())/float(RAND_MAX)) * 0.4);
    ROS_INFO_STREAM("Random x position: " << model.pose.position.x);
    model.pose.position.z = 0.11;

    model.pose.orientation.x = 0.0;
    model.pose.orientation.y = 0.0;
    model.pose.orientation.z = 0.0;
    model.pose.orientation.w = 0.0;

    model.twist.linear.x = 0.0;
    model.twist.linear.y = 0.0;
    model.twist.linear.z = 0.0;

    model.twist.angular.x = 0.0;
    model.twist.angular.y = 0.0;
    model.twist.angular.z = 0.0;

    model.reference_frame = "world";

    pub.publish(model);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;

  }

  return 0;

}
