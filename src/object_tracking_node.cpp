#include "ros/ros.h"
#include "ObjectTracker.h"


#include <vector>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_tracking_node");

  ros::NodeHandle nh;
  double frequency = 100.0;


  // Parameters
  // std::string topic_wrench_external;
  // std::string topic_wrench_control;
  // std::string topic_desired_equilibrium;;

  // double M_object;
  // double Z_ceiling;
  // double Z_level;


  // if (!nh.getParam("topic_wrench_external", topic_wrench_external))   {
  //   ROS_ERROR("Couldn't retrieve the topic name for the external wrench. ");
  //   return -1;
  // }

  // if (!nh.getParam("topic_wrench_control", topic_wrench_control))   {
  //   ROS_ERROR("Couldn't retrieve the topic name for the control wrench. ");
  //   return -1;
  // }

  // if (!nh.getParam("topic_desired_equilibrium", topic_desired_equilibrium))   {
  //   ROS_ERROR("Couldn't retrieve the topic name for the desired equilibrium point. ");
  //   return -1;
  // }

  // if (!nh.getParam("M_object", M_object))   {
  //   ROS_ERROR("Couldn't retrieve mass of the object. ");
  //   return -1;
  // }

  // if (!nh.getParam("Z_ceiling", Z_ceiling))   {
  //   ROS_ERROR("Couldn't retrieve mass of the object. ");
  //   return -1;
  // }

  // if (!nh.getParam("Z_level", Z_level))   {
  //   ROS_ERROR("Couldn't retrieve mass of the object. ");
  //   return -1;
  // }


  ObjectTracker object_tracker(nh, frequency);

  // load_support_controller.Run();


  return 0;
}