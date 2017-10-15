#include "ros/ros.h"
#include "LoadSupportController.h"


#include <vector>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "load_support_node");

  ros::NodeHandle nh;
  double frequency = 300.0;


  // Parameters
  std::string wrench_external_topic_name;
  std::string wrench_control_topic_name;

  double M_object;



  if (!nh.getParam("wrench_external_topic_name", wrench_external_topic_name))   {
    ROS_ERROR("Couldn't retrieve the topic name for the external wrench. ");
    return -1;
  }

  if (!nh.getParam("wrench_control_topic_name", wrench_control_topic_name))   {
    ROS_ERROR("Couldn't retrieve the topic name for the control wrench. ");
    return -1;
  }

  if (!nh.getParam("M_object", M_object))   {
    ROS_ERROR("Couldn't retrieve mass of the object. ");
    return -1;
  }


  LoadSupportController load_support_controller(nh, frequency,
      M_object,
      wrench_external_topic_name,
      wrench_control_topic_name);

  // load_support_controller.Run();


  return 0;
}