#ifndef __LOADSUPPORTCONTROLLER__
#define __LOADSUPPORTCONTROLLER__

#include "ros/ros.h"
// #include "geometry_msgs/Pose.h"
// #include "geometry_msgs/Twist.h"
// #include "geometry_msgs/TwistStamped.h"
// #include "geometry_msgs/PointStamped.h"
// #include "nav_msgs/Path.h"


// #include <vector>

#include "geometry_msgs/WrenchStamped.h"


#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"


// #include <mutex>

using namespace Eigen;

typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;


const double gravity_acc = 9.80665;


class LoadSupportController {


private:

	// ROS variables
	ros::NodeHandle nh_;
	ros::Rate loop_rate_;
	double dt_;

	ros::Subscriber sub_platform_state_;


	std::string wrench_external_topic_name_;
	std::string wrench_control_topic_name_;

	//Controller variables
	double M_object_;
	double M_estiamted_;
	double loadShare_;


	// INPUT SIGNAL
	// external wrench (force/torque sensor) in "????" frame
	Vector6d wrench_external_;

	// OUTPUT SIGNAL
	// control wrench (from any controller) expected to be in "????" frame
	Vector6d wrench_control_;


public:
	LoadSupportController(ros::NodeHandle &n,
	                      double frequency,
	                      double M_object,
	                      std::string wrench_external_topic_name,
	                      std::string wrench_control_topic_name);


	// void Run();

private:

	// bool InitializeDS();

	// bool InitializeROS();

	void UpdateExternalForce(const geometry_msgs::WrenchStamped::ConstPtr& msg);

	void ComputeLoadShare();

	// void ComputeDesiredVelocity();

	// void PublishDesiredVelocity();

	// void PublishFuturePath();

};


#endif //__LOADSUPPORTCONTROLLER__