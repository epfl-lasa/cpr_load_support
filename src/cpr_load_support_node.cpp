#include "LoadSupportController.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "load_support_node");

  ros::NodeHandle nh;
  double frequency = 300.0;

  // Parameters
  std::string topic_wrench_external;
  std::string topic_wrench_control;
  std::string topic_desired_equilibrium;

  std::string frame_arm_base;
  std::string frame_arm_endeffector;
  std::string frame_object;


  double M_object;
  double Max_weight_compensation;
  double Z_ceiling;
  double Z_level;

  double talking_rate;
  double talking_forced_rate;

  double loadShare_trigger;
  double loadShare_final;
  double loadShare_contact;
  // check for validiting of the load-share


  std::vector<double> ee_rest_position = {0.111, 0.494 ,0.587};
  // std::vector<double> ee_rest_position = {0.111, 0.494, 0.75};



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

  if (!nh.getParam("frame_arm_base", frame_arm_base))   {
    ROS_ERROR("Couldn't retrieve the frame id for the base of the arm ");
    return -1;
  }

  if (!nh.getParam("frame_arm_endeffector", frame_arm_endeffector))   {
    ROS_ERROR("Couldn't retrieve the frame id for the end-effector of the arm ");
    return -1;
  }

  if (!nh.getParam("frame_object", frame_object))   {
    ROS_ERROR("Couldn't retrieve the frame id for the object ");
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

  if (!nh.getParam("Max_weight_compensation", Max_weight_compensation))   {
    ROS_ERROR("Couldn't retrieve the maximum for weight compensation ");
    return -1;
  }


  if (!nh.getParam("talking_rate", talking_rate))   {
    ROS_ERROR("Couldn't retrieve the pause time between spoken statements");
    return -1;
  }

  if (!nh.getParam("talking_forced_rate", talking_forced_rate))   {
    ROS_ERROR("Couldn't retrieve the maximum silent time");
    return -1;
  }

  if (!nh.getParam("loadShare_trigger", loadShare_trigger))   {
    ROS_ERROR("Couldn't retrieve the load-share-trigger");
    return -1;
  }

  if (!nh.getParam("loadShare_final", loadShare_final))   {
    ROS_ERROR("Couldn't retrieve the load-share-trigger");
    return -1;
  }

  if (!nh.getParam("loadShare_contact", loadShare_contact))   {
    ROS_ERROR("Couldn't retrieve the load-share-trigger");
    return -1;
  }



  // check if the values for laod-share are valid


  if (loadShare_trigger < 0 || loadShare_trigger > 1) {
    ROS_ERROR_STREAM("Loadshare-trigger should be between 0 and 1.");
    return -1;
  }

  if (loadShare_final <= loadShare_trigger || loadShare_final > 1) {
    ROS_ERROR_STREAM("Loadshare-final should be between " <<  loadShare_trigger << " and 1.");
    return -1;
  }

  if (loadShare_contact < 0 || loadShare_contact > loadShare_trigger) {
    ROS_ERROR_STREAM("Loadshare-contact should be between 0 and " <<  loadShare_trigger );
    return -1;
  }



  LoadSupportController load_support_controller(
    nh,
    frequency,
    M_object,
    Z_ceiling,
    Z_level,
    Max_weight_compensation,
    talking_rate,
    talking_forced_rate,
    loadShare_trigger,
    loadShare_final,
    loadShare_contact,
    ee_rest_position,
    topic_wrench_external,
    topic_wrench_control,
    topic_desired_equilibrium,
    frame_arm_base,
    frame_arm_endeffector,
    frame_object);

  load_support_controller.Run();


  return 0;
}