#include "ros/ros.h"
#include "ObjectTracker.h"


#include <vector>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_tracking_node");

  ros::NodeHandle nh;
  double frequency = 100.0;


  // Parameters
  std::string frame_robot_static;
  std::string frame_robot_calibration;
  std::string frame_robot_object;
  std::string frame_robot_base;
  std::string frame_mocap_static;
  std::string frame_mocap_object;



  if (!nh.getParam("frame_robot_static", frame_robot_static))   {
    ROS_ERROR("Couldn't retrieve the frame id for the static marker on the robot (eg. top_right_corner). ");
    return -1;
  }

  if (!nh.getParam("frame_robot_calibration", frame_robot_calibration))   {
    ROS_ERROR("Couldn't retrieve the frame id for the calibration tf (eg. top_left_corner). ");
    return -1;
  }

  if (!nh.getParam("frame_robot_object", frame_robot_object))   {
    ROS_ERROR("Couldn't retrieve the frame id for the object to be published on (eg. object). ");
    return -1;
  }

  if (!nh.getParam("frame_robot_base", frame_robot_base))   {
    ROS_ERROR("Couldn't retrieve the frame id for the base of the robot (eg. base_link). ");
    return -1;
  }

  if (!nh.getParam("frame_mocap_static", frame_mocap_static))   {
    ROS_ERROR("Couldn't retrieve the MOCAP id for the static marker on the robot (eg. mocap_top_right_corner). ");
    return -1;
  }

  if (!nh.getParam("frame_mocap_object", frame_mocap_object))   {
    ROS_ERROR("Couldn't retrieve the MOCAP id for the object to be tracked (eg. mocap_object). ");
    return -1;
  }




  // std::string frame_robot_static      = "top_right_corner";
  // std::string frame_robot_calibration = "top_left_corner";
  // std::string frame_robot_object      = "object";
  // std::string frame_robot_base        = "base_link";
  // std::string frame_mocap_static      = "mocap_top_right_corner";
  // std::string frame_mocap_object      = "mocap_object";



  ObjectTracker object_tracker(nh, frequency,
                               frame_robot_static,
                               frame_robot_calibration,
                               frame_robot_object,
                               frame_robot_base,
                               frame_mocap_static,
                               frame_mocap_object);

  object_tracker.Run();


  return 0;
}