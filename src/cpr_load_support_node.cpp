#include "ros/ros.h"
#include "LoadSupportController.h"


#include <vector>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "load_support_node");

  ros::NodeHandle nh;
  double frequency = 300.0;


  // Parameters
  std::string topic_wrench_external;
  std::string topic_wrench_control;
  std::string topic_desired_equilibrium;;

  double M_object;
  double Z_ceiling;
  double Z_level;


  if (!nh.getParam("topic_wrench_external", topic_wrench_external))   {
    ROS_ERROR("Couldn't retrieve the topic name for the external wrench. ");
    return -1;
  }

  if (!nh.getParam("topic_wrench_control", topic_wrench_control))   {
    ROS_ERROR("Couldn't retrieve the topic name for the control wrench. ");
    return -1;
  }

  if (!nh.getParam("topic_desired_equilibrium", topic_desired_equilibrium))   {
    ROS_ERROR("Couldn't retrieve the topic name for the desired equilibrium point. ");
    return -1;
  }

  if (!nh.getParam("M_object", M_object))   {
    ROS_ERROR("Couldn't retrieve mass of the object. ");
    return -1;
  }

  if (!nh.getParam("Z_ceiling", Z_ceiling))   {
    ROS_ERROR("Couldn't retrieve mass of the object. ");
    return -1;
  }

  if (!nh.getParam("Z_level", Z_level))   {
    ROS_ERROR("Couldn't retrieve mass of the object. ");
    return -1;
  }


  double Max_weight_compensation = 15;
  std::vector<double> ee_rest_position = {0.111, 0.494, 0.587};


  LoadSupportController load_support_controller(nh, frequency,
      M_object,
      Z_ceiling,
      Z_level,
      Max_weight_compensation,
      ee_rest_position,
      topic_wrench_external,
      topic_wrench_control,
      topic_desired_equilibrium);

  load_support_controller.Run();


  return 0;
}