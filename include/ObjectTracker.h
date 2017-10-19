#ifndef __OBJECTTRACKER__
#define __OBJECTTRACKER__

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <iostream>


class ObjectTracker {


private:

	// ROS variables
	ros::NodeHandle nh_;
	ros::Rate loop_rate_;

	tf::TransformListener listener_;


public:
	ObjectTracker(ros::NodeHandle &n,
	                      double frequency);


private:



};


#endif //__OBJECTTRACKER__